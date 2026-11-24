// One mono8 ROS image, one full-frame multi-symbol ZXing pass. Release the
// GIL while decoding so that both cameras and the flight guards run independently.
#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include <ZXing/ReadBarcode.h>
#include "barcode_reader_camera/config_yaml.hpp"
#include <cmath>
#include <exception>
#include <memory>
#include <stdexcept>
#include <vector>
#include <string>

static PyObject* decode(PyObject*, PyObject* args)
{
    const char* data;
    Py_ssize_t size;
    int width, height, stride;
    if (!PyArg_ParseTuple(args, "y#iii", &data, &size, &width, &height, &stride))
        return nullptr;
    if (width <= 0 || height <= 0 || stride < width || int64_t(stride) * height != size) {
        PyErr_SetString(PyExc_ValueError, "invalid mono8 image dimensions");
        return nullptr;
    }
    std::vector<std::string> codes;
    auto* state = PyEval_SaveThread();
    try {
        ZXing::ImageView image(reinterpret_cast<const uint8_t*>(data), width, height,
                               ZXing::ImageFormat::Lum, stride);
        ZXing::ReaderOptions options;
        options.setFormats(ZXing::BarcodeFormat::QRCode).setTryHarder(true)
               .setTryRotate(true).setMaxNumberOfSymbols(0);
        for (const auto& code : ZXing::ReadBarcodes(image, options))
            if (code.isValid()) codes.push_back(code.text());
    } catch (...) {
        PyEval_RestoreThread(state);
        PyErr_SetString(PyExc_RuntimeError, "ZXing failed to read mono8 image");
        return nullptr;
    }
    PyEval_RestoreThread(state);
    auto* result = PyList_New(0);
    if (!result) return nullptr;
    for (const auto& code : codes) {
        auto* text = PyUnicode_DecodeUTF8(code.data(), code.size(), "strict");
        if (!text || PyList_Append(result, text) < 0) {
            Py_XDECREF(text);
            Py_DECREF(result);
            return nullptr;
        }
        Py_DECREF(text);
    }
    return result;
}

namespace {
using namespace barcode_reader_camera;
constexpr char reader_type[] = "orange_qr.LabelReader";

struct LabelReader {
    DecoderConfig config;
    CameraModel camera;
};

void destroy_reader(PyObject* capsule)
{
    delete static_cast<LabelReader*>(PyCapsule_GetPointer(capsule, reader_type));
}

bool read_doubles(PyObject* sequence, std::vector<double>& values)
{
    PyObject* items = PySequence_Fast(sequence, "camera parameters must be a sequence");
    if (!items) return false;
    for (Py_ssize_t i = 0; i < PySequence_Fast_GET_SIZE(items); ++i) {
        double item = PyFloat_AsDouble(PySequence_Fast_GET_ITEM(items, i));
        if (PyErr_Occurred() || !std::isfinite(item)) {
            if (!PyErr_Occurred()) PyErr_SetString(PyExc_ValueError, "nonfinite camera parameters");
            Py_DECREF(items);
            return false;
        }
        values.push_back(item);
    }
    Py_DECREF(items);
    return true;
}

PyObject* label_reader(PyObject*, PyObject* args)
{
    const char *config_path, *layout_path;
    PyObject *intrinsics, *distortion;
    if (!PyArg_ParseTuple(args, "ssOO", &config_path, &layout_path, &intrinsics, &distortion))
        return nullptr;
    try {
        std::vector<double> k, d;
        if (!read_doubles(intrinsics, k) || !read_doubles(distortion, d)) return nullptr;
        if (k.size() != 9 || k[0] <= 0 || k[4] <= 0 || k[8] != 1 ||
            !(d.empty() || d.size() == 4 || d.size() == 5 || d.size() == 8 ||
              d.size() == 12 || d.size() == 14))
            throw std::invalid_argument("invalid camera intrinsics or distortion");
        std::unique_ptr<LabelReader> reader(new LabelReader);
        reader->camera.camera_matrix = cv::Mat(3, 3, CV_64F, k.data()).clone();
        if (!d.empty()) reader->camera.dist_coeffs = cv::Mat(d).clone();
        reader->config = loadDecoderConfig(YAML::LoadFile(config_path));
        applyLayout(YAML::LoadFile(layout_path), reader->config, nullptr);
        validateDecoderConfig(reader->config);
        // The paired window supplies each subsequent frame together with its own pose.
        reader->config.retry_timeout_ms = 0;
        PyObject* capsule = PyCapsule_New(reader.get(), reader_type, destroy_reader);
        if (capsule) reader.release();
        return capsule;
    } catch (const std::exception& e) {
        PyErr_SetString(PyExc_ValueError, e.what());
        return nullptr;
    }
}

PyObject* decode_label(PyObject*, PyObject* args)
{
    PyObject* capsule;
    const char* data;
    Py_ssize_t size;
    int width, height, stride;
    TagPose pose;
    if (!PyArg_ParseTuple(args, "Oy#iii(ddd)(ddd)", &capsule, &data, &size, &width, &height, &stride,
                          &pose.rvec[0], &pose.rvec[1], &pose.rvec[2], &pose.tvec[0], &pose.tvec[1],
                          &pose.tvec[2])) return nullptr;
    auto* reader = static_cast<LabelReader*>(PyCapsule_GetPointer(capsule, reader_type));
    if (!reader) return nullptr;
    if (width <= 0 || height <= 0 || stride < width || int64_t(stride) * height != size ||
        !cv::checkRange(pose.rvec) || !cv::checkRange(pose.tvec) || pose.tvec[2] <= 0) {
        PyErr_SetString(PyExc_ValueError, "invalid label image or pose");
        return nullptr;
    }
    DecodeOutput result;
    auto* state = PyEval_SaveThread();
    try {
        const cv::Mat gray(height, width, CV_8UC1, const_cast<char*>(data), stride);
        result = decodeBarcodes(gray, &pose, reader->camera, reader->config);
    } catch (const std::exception& e) {
        PyEval_RestoreThread(state);
        PyErr_SetString(PyExc_RuntimeError, e.what());
        return nullptr;
    }
    PyEval_RestoreThread(state);
    PyObject* codes = PyList_New(0);
    if (!codes) return nullptr;
    for (const auto& code : result.codes) {
        PyObject* entry = Py_BuildValue("(s#s)", code.code.data(), Py_ssize_t(code.code.size()),
                                        code.source.c_str());
        if (!entry || PyList_Append(codes, entry) < 0) {
            Py_XDECREF(entry);
            Py_DECREF(codes);
            return nullptr;
        }
        Py_DECREF(entry);
    }
    return codes;
}
}  // namespace

static PyMethodDef methods[] = {
    {"label_reader", label_reader, METH_VARARGS, "Load the label layout and calibrated camera model."},
    {"decode_label", decode_label, METH_VARARGS, "Decode one pose-matched label, returning (text, strategy) pairs."},
    {"decode", decode, METH_VARARGS, "Decode QR texts from mono8 data, width, height, stride."},
    {nullptr, nullptr, 0, nullptr}
};
static PyModuleDef module = {PyModuleDef_HEAD_INIT, "orange_qr", nullptr, -1, methods,
                             nullptr, nullptr, nullptr, nullptr};
PyMODINIT_FUNC PyInit_orange_qr() { return PyModule_Create(&module); }

// One mono8 ROS image, one full-frame multi-symbol ZXing pass. Release the
// GIL while decoding so that both cameras and the flight guards run independently.
#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include <ZXing/ReadBarcode.h>
#include <exception>
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

static PyMethodDef methods[] = {
    {"decode", decode, METH_VARARGS, "Decode QR texts from mono8 data, width, height, stride."},
    {nullptr, nullptr, 0, nullptr}
};
static PyModuleDef module = {PyModuleDef_HEAD_INIT, "orange_qr", nullptr, -1, methods,
                             nullptr, nullptr, nullptr, nullptr};
PyMODINIT_FUNC PyInit_orange_qr() { return PyModule_Create(&module); }

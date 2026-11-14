import {
  ArrowHelper, BufferGeometry, Color, CylinderGeometry, DoubleSide, Float32BufferAttribute,
  Group, Mesh, MeshBasicMaterial, Points, ShaderMaterial, SphereGeometry, TorusGeometry, Vector3,
} from "three";
import { Line2 } from "three/addons/lines/Line2.js";
import { LineGeometry } from "three/addons/lines/LineGeometry.js";
import { LineMaterial } from "three/addons/lines/LineMaterial.js";

export function makeCloud(positions, color, pixels, opacity) {
  const geometry = new BufferGeometry();
  geometry.setAttribute("position", new Float32BufferAttribute(positions, 3));
  const material = new ShaderMaterial({
    uniforms: {
      tint: { value: new Color(color) },
      pixels: { value: pixels * Math.min(devicePixelRatio, 2) },
      opacity: { value: opacity },
    },
    vertexShader: `
      uniform float pixels;
      void main() {
        gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
        gl_PointSize = pixels;
      }
    `,
    fragmentShader: `
      uniform vec3 tint;
      uniform float opacity;
      void main() {
        float d = length(gl_PointCoord - vec2(0.5));
        if (d > 0.5) discard;
        gl_FragColor = vec4(tint, opacity * (1.0 - smoothstep(0.32, 0.5, d)));
        #include <tonemapping_fragment>
        #include <colorspace_fragment>
      }
    `,
    transparent: true,
    depthWrite: false,
  });
  const points = new Points(geometry, material);
  points.renderOrder = -2;
  return points;
}

export function wideLine(points, options) {
  const geometry = new LineGeometry().setPositions(points);
  const material = new LineMaterial(options);
  const line = new Line2(geometry, material);
  if (options.dashed) line.computeLineDistances();
  return line;
}

export function droneModel() {
  const model = new Group();
  const bodyForward = new Vector3(1, 0, 0);
  const hull = new MeshBasicMaterial({ color: 0x7deaf3 });
  const arms = new MeshBasicMaterial({ color: 0xa0c0cd });
  const rotors = new MeshBasicMaterial({ color: 0xe4eff7, side: DoubleSide });
  const body = new Mesh(new SphereGeometry(.041, 16, 10), hull);
  body.scale.set(1.18, .88, .55);
  model.add(body);
  for (const [x, y] of [[.054, .054], [.054, -.054], [-.054, .054], [-.054, -.054]]) {
    const arm = new Mesh(new CylinderGeometry(.004, .004, Math.hypot(x, y), 8), arms);
    arm.position.set(x / 2, y / 2, 0);
    arm.quaternion.setFromUnitVectors(new Vector3(0, 1, 0), new Vector3(x, y, 0).normalize());
    const rotor = new Mesh(new TorusGeometry(.026, .0027, 6, 24), rotors);
    rotor.position.set(x, y, .007);
    model.add(arm, rotor);
  }
  const nose = new ArrowHelper(bodyForward, new Vector3(.035, 0, .025), .24, 0xf3f7fb, .072, .061);
  model.add(nose);
  return model;
}

(function (root, factory) {
  const api = factory();
  if (typeof module === "object" && module.exports) module.exports = api;
  if (root) root.StructuralDiagnostics = api;
})(typeof globalThis !== "undefined" ? globalThis : this, function () {
  "use strict";

  function finiteVector(value, length, name) {
    if (!Array.isArray(value) && !ArrayBuffer.isView(value)) {
      throw new TypeError(`${name} must be an array-like ${length}-vector`);
    }
    if (value.length !== length) {
      throw new RangeError(`${name} must contain exactly ${length} values`);
    }
    const result = Array.from(value, Number);
    if (!result.every(Number.isFinite)) {
      throw new RangeError(`${name} must contain only finite values`);
    }
    return result;
  }

  function positiveProperty(section, longName, shortName) {
    if (!section || typeof section !== "object") {
      throw new TypeError("section must provide area, I, and J properties");
    }
    const value = Number(section[longName] ?? section[shortName]);
    if (!Number.isFinite(value) || value <= 0) {
      throw new RangeError(`${longName} must be a positive finite value`);
    }
    return value;
  }

  function resultant(force3, moment3) {
    const [fx, fy, fz] = finiteVector(force3, 3, "force3");
    const [mx, my, mz] = finiteVector(moment3, 3, "moment3");
    return {
      axial_n: fz,
      shear_n: Math.hypot(fx, fy),
      bending_nm: Math.hypot(mx, my),
      torsion_nm: mz,
      force_norm_n: Math.hypot(fx, fy, fz),
      moment_norm_nm: Math.hypot(mx, my, mz),
    };
  }

  function sectionPoint(point3, force3, moment3, section) {
    const [x, y, z] = finiteVector(point3, 3, "point3");
    const [fx, fy, fz] = finiteVector(force3, 3, "force3");
    const [mx0, my0, mz] = finiteVector(moment3, 3, "moment3");
    const area = positiveProperty(section, "area_m2", "A");
    const inertia = positiveProperty(section, "I_m4", "I");
    const polarInertia = positiveProperty(section, "J_m4", "J");

    // Transport the section-origin moment to the axial station z.
    const mx = mx0 + z * fy;
    const my = my0 - z * fx;
    const axial = fz / area;
    const bending = (mx * y - my * x) / inertia;
    const shearX = fx / area;
    const shearY = fy / area;
    const torsionX = -mz * y / polarInertia;
    const torsionY = mz * x / polarInertia;
    const normal = axial + bending;
    const total = Math.hypot(normal, Math.sqrt(3) * (shearX + torsionX), Math.sqrt(3) * (shearY + torsionY));

    return {
      axial_pa: axial,
      bending_pa: bending,
      shear_x_pa: shearX,
      shear_y_pa: shearY,
      torsion_x_pa: torsionX,
      torsion_y_pa: torsionY,
      normal_pa: normal,
      total_pa: total,
      axial_vm_pa: Math.abs(axial),
      bending_vm_pa: Math.abs(bending),
      shear_vm_pa: Math.sqrt(3) * Math.hypot(shearX, shearY),
      torsion_vm_pa: Math.sqrt(3) * Math.hypot(torsionX, torsionY),
      station_moment_nm: [mx, my, mz],
    };
  }

  function femTensorMetrics(voigt6) {
    const [xx, yy, zz, xy, yz, zx] = finiteVector(voigt6, 6, "voigt6");
    const normalDeviatoricSquared = 0.5 * (
      (xx - yy) ** 2 + (yy - zz) ** 2 + (zz - xx) ** 2
    );
    const shearSquared = 3 * (xy ** 2 + yz ** 2 + zx ** 2);
    return {
      von_mises_pa: Math.sqrt(normalDeviatoricSquared + shearSquared),
      hydrostatic_pa: (xx + yy + zz) / 3,
      normal_deviatoric_vm_pa: Math.sqrt(normalDeviatoricSquared),
      shear_vm_pa: Math.sqrt(shearSquared),
    };
  }

  return Object.freeze({ resultant, sectionPoint, femTensorMetrics });
});

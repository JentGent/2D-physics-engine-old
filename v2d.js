const V2D = (() => ({
    Add(v1, v2) {
        return [v1[0] + v2[0], v1[1] + v2[1]];
    },
    Sub(v1, v2) {
        return [v1[0] - v2[0], v1[1] - v2[1]];
    },
    Mult(v1, v2) {
        return [v1[0] * v2[0], v1[1] * v2[1]];
    },
    Scale(v, s) {
        return [v[0] * s, v[1] * s];
    },
    Normalize(v) {
        const d = 1 / Math.sqrt(v[0] * v[0] + v[1] * v[1]);
        return [v[0] * d, v[1] * d];
    },
    Rotate(v, origin, angle) {
        const c = Math.cos(angle), s = Math.sin(angle), dx = v[0] - origin[0], dy = v[1] - origin[1];
        return [origin[0] + c * dx - s * dy, origin[1] + c * dy + s * dx];
    },
    iAdd(v1, v2) {
        v1[0] += v2[0];
        v1[1] += v2[1];
        return v1;
    },
    iSub(v1, v2) {
        v1[0] -= v2[0];
        v1[1] -= v2[1];
        return v1;
    },
    iMult(v1, v2) {
        v1[0] *= v2[0];
        v1[1] *= v2[1];
        return v1;
    },
    iScale(v, s) {
        v[0] *= s;
        v[1] *= s;
        return v;
    },
    iNormalize(v) {
        const d = 1 / Math.sqrt(v[0] * v[0] + v[1] * v[1]);
        v[0] *= d;
        v[1] *= d;
        return v;
    },
    iRotate(v, origin, angle) {
        const c = Math.cos(angle), s = Math.sin(angle), dx = v[0] - origin[0], dy = v[1] - origin[1];
        v[0] = origin[0] + c * dx - s * dy;
        v[1] = origin[1] + c * dy + s * dx;
        return v;
    },
    Mag(v) {
        return Math.sqrt(v[0] * v[0] + v[1] * v[1]);
    },
    SqMag(v) {
        return v[0] * v[0] + v[1] * v[1];
    },
    Dot(v1, v2) {
        return v1[0] * v2[0] + v1[1] * v2[1];
    },
    Cross(v1, v2) {
        return v1[0] * v2[1] - v1[1] * v2[0];
    }
}))();
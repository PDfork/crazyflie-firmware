#pragma once

#include <math.h>
#include <string.h> // for memset()


#define GRAV 9.81
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define AS_1D(x) (&(x)[0][0])

#define ZEROARR(a) memset(a, 0, sizeof(a));

void fzero(float *f, int sz)
{
	for (int i = 0; i < sz; ++i) {
		f[i] = 0;
	}
}

void d2f(double const *d, float *f, int n)
{
	for (int i = 0; i < n; ++i) {
		f[i] = d[i];
	}
}

void f2d(float const *f, double *d, int n)
{
	for (int i = 0; i < n; ++i) {
		d[i] = f[i];
	}
}

float fsqr(float x) { return x * x; }

float radians(float degrees) { return (M_PI / 180.0) * degrees; }
float degrees(float radians) { return (180.0 / M_PI) * radians; }


// ---------------------------- 3d vectors ------------------------------

struct vec { 
	float x; float y; float z;
};

// constructors
struct vec mkvec(float x, float y, float z) {
	struct vec v = { .x = x, .y = y, .z = z };
	return v;
}
struct vec vrepeat(float x) { 
	return mkvec(x, x, x); 
}
struct vec vzero() { 
	return vrepeat(0.0f);
}
struct vec float2vec(float const v[3]) {
	return mkvec(v[0], v[1], v[2]);
}


// linear operators
struct vec vscl(float s, struct vec v) {
	return mkvec(s * v.x , s * v.y, s * v.z);
}
struct vec vneg(struct vec v) {
	return mkvec(-v.x, -v.y, -v.z);
}
struct vec vdiv(struct vec v, float s) {
	return vscl(1.0f/s, v);
}
struct vec vadd(struct vec a, struct vec b) {
	return mkvec(a.x + b.x, a.y + b.y, a.z + b.z);
}
struct vec vsub(struct vec a, struct vec b) {
	return vadd(a, vneg(b));
}
// in-place operators
//void vaddi(struct vec *a, struct vec b) {
	//*a = vadd(*a, b);
//}

// other operators
float vdot(struct vec a, struct vec b) {
	return a.x * b.x + a.y * b.y + a.z * b.z;
}
float vmag2(struct vec v) {
	return vdot(v, v);
}
float vmag(struct vec v) {
	return sqrt(vmag2(v)); 
}
struct vec vnormalized(struct vec v) {
	return vdiv(v, vmag(v));
}
struct vec vcross(struct vec a, struct vec b) {
	return mkvec(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x);
}

// special functions to ease the pain of writing vector math in C :)
struct vec vadd3(struct vec a, struct vec b, struct vec c) {
	return vadd(vadd(a, b), c);
}
struct vec vsub2(struct vec a, struct vec b, struct vec c) {
	return vadd3(a, vneg(b), vneg(c));
}

struct vec vload(double const *d)
{
	return mkvec(d[0], d[1], d[2]);
}
void vstore(struct vec v, double *d)
{
	d[0] = v.x; d[1] = v.y; d[2] = v.z;
}
struct vec vloadf(float const *f)
{
	return mkvec(f[0], f[1], f[2]);
}
void vstoref(struct vec v, float *f)
{
	f[0] = v.x; f[1] = v.y; f[2] = v.z;
}

// comparison
bool veq(struct vec a, struct vec b)
{
	return (a.x == b.x) && (a.y == b.y) && (a.z == b.z);
}
bool vneq(struct vec a, struct vec b)
{
	return !veq(a, b);
}


// ---------------------------- 3x3 matrices ------------------------------

struct mat33 { 
	float m[3][3];
};

// constructors
struct mat33 mzero() {
	struct mat33 m;
	fzero(AS_1D(m.m), 9);
	return m;
}
struct mat33 diag(float a, float b, float c) {
	struct mat33 m = mzero();
	m.m[0][0] = a;
	m.m[1][1] = b;
	m.m[2][2] = c;
	return m;
}
struct mat33 eyescl(float a) { 
	return diag(a, a, a);
}
struct mat33 eye() {
	return eyescl(1.0f);
}

// just the very simple ops needed for ekf
struct mat33 mmult(struct mat33 a, struct mat33 b) {
	struct mat33 ab;
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			float accum = 0;
			for (int k = 0; k < 3; ++k) {
				accum += a.m[i][k] * b.m[k][j];
			}
			ab.m[i][j] = accum;
		}
	}
	return ab;
}
struct mat33 mscale(float s, struct mat33 a) {
	// TODO make sure it gets optimized into one loop of 9, not a nested loop
	struct mat33 sa;
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			sa.m[i][j] = s * a.m[i][j];
		}
	}
	return sa;
}
struct mat33 maddridge(struct mat33 a, float d) {
	a.m[0][0] += d;
	a.m[1][1] += d;
	a.m[2][2] += d;
	return a;
}
// matrix A(v) such that Ax = v cross x
struct mat33 crossmat(struct vec v) {
	struct mat33 m;
	m.m[0][0] = 0;
	m.m[0][1] = -v.z;
	m.m[0][2] = v.y;
	m.m[1][0] = v.z;
	m.m[1][1] = 0;
	m.m[1][2] = -v.x;
	m.m[2][0] = -v.y;
	m.m[2][1] = v.x;
	m.m[2][2] = 0;
	return m;
}
// set a 3x3 block within a big matrix
void set_block33(float *block, int stride, struct mat33 m) {
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			block[j] = m.m[i][j];
		}
		block += stride;
	}
}


// ---------------------------- quaternions ------------------------------

struct quat { 
	union {
		struct vec v; 
		struct {
			float x;
			float y;
			float z;
		};
	};
	float w; 
};

// constructors
struct quat mkquat(float x, float y, float z, float w) {
	struct quat q = { .v = mkvec(x, y, z), .w = w };
	return q;
}
struct quat quatvw(struct vec v, float w) {
	struct quat q = { .v = v, .w = w };
	return q;
}
struct quat qeye() {
	return mkquat(0, 0, 0, 1);
}
struct quat qaxisangle(struct vec axis, float angle) {
	float scale = sin(angle / 2) / vmag(axis);
	struct quat q = { .v = vscl(scale, axis), .w = cos(angle/2) };
	return q;
}
struct quat qrpy_small(struct vec rpy) {
	float q2 = vmag2(rpy) / 4.0f;
	if (q2 < 1) {
		return quatvw(vdiv(rpy, 2), sqrtf(1.0f - q2));
	} 
	else {
		float w = 1.0f / sqrtf(1.0f + q2);
		return quatvw(vscl(w/2, rpy), w);
	}
}

struct quat qload(double const *d)
{
	return mkquat(d[0], d[1], d[2], d[3]);
}
void qstore(struct quat q, double *d)
{
	d[0] = q.x; d[1] = q.y; d[2] = q.z; d[3] = q.w;
}
struct quat qloadf(float const *f)
{
	return mkquat(f[0], f[1], f[2], f[3]);
}
void qstoref(struct quat q, float *f)
{
	f[0] = q.x; f[1] = q.y; f[2] = q.z; f[3] = q.w;
}


// operations
struct vec qvrot(struct quat q, struct vec v) {
	// from http://gamedev.stackexchange.com/a/50545 - TODO find real citation
	return vadd3(
		vscl(2.0f * vdot(q.v, v), q.v), 
		vscl(q.w * q.w - vdot(q.v, q.v), v),
		vscl(2.0f * q.w, vcross(q.v, v))
	);
}
struct quat qqmul(struct quat q, struct quat p) {
	float x =  q.w*p.x + q.z*p.y - q.y*p.z + q.x*p.w;
	float y = -q.z*p.x + q.w*p.y + q.x*p.z + q.y*p.w;
	float z =  q.y*p.x - q.x*p.y + q.w*p.z + q.z*p.w;
	float w = -q.x*p.x - q.y*p.y - q.z*p.z + q.w*p.w;
	return mkquat(x, y, z, w);
}
struct quat qinv(struct quat q) {
	return quatvw(vneg(q.v), q.w);
}
struct quat qnormalized(struct quat q) {
	float maginv = 1.0f / sqrtf(vmag2(q.v) + fsqr(q.w));
	return quatvw(vscl(maginv, q.v), maginv * q.w);
}
struct quat quat_gyro_update(struct quat quat, struct vec gyro, float const dt) {
	// from "Indirect Kalman Filter for 3D Attitude Estimation", N. Trawny, 2005
	struct quat q1;
	double const r = (dt / 2) * gyro.x;
	double const p = (dt / 2) * gyro.y;
	double const y = (dt / 2) * gyro.z;

	q1.v.x =    quat.x + y*quat.y - p*quat.z + r*quat.w;
	q1.v.y = -y*quat.x +   quat.y + r*quat.z + p*quat.w;
	q1.v.z =  p*quat.x - r*quat.y +   quat.z + y*quat.w;
	q1.w   = -r*quat.x - p*quat.y - y*quat.z +   quat.w;
	return q1;
}
struct mat33 quat2rotmat(struct quat q) {
	float x = q.x;
	float y = q.y;
	float z = q.z;
	float w = q.w;

	struct mat33 m;
	m.m[0][0] = 1 - 2*y*y - 2*z*z;
	m.m[0][1] = 2*x*y - 2*z*w;
	m.m[0][2] = 2*x*z + 2*y*w,
	m.m[1][0] = 2*x*y + 2*z*w;
	m.m[1][1] = 1 - 2*x*x - 2*z*z;
	m.m[1][2] = 2*y*z - 2*x*w,
	m.m[2][0] = 2*x*z - 2*y*w;
	m.m[2][1] = 2*y*z + 2*x*w;
	m.m[2][2] = 1 - 2*x*x - 2*y*y;
	return m;
}

// big matrices

#define IND(A, m, n, r, c) ((A)[(r) * (n) + (c)])
#define INDT(A, m, n, r, c) IND(A, n, m, c, r)

#define SGEMM_LOOP(IA, IB) \
for (int i = 0; i < m; ++i) { \
	for (int j = 0; j < n; ++j) { \
		float accum = 0; \
		for (int w = 0; w < k; ++w) { \
			accum += IA(a, m, k, i, w) * IB(b, k, n, w, j); \
		} \
		IND(c, m, n, i, j) = beta * IND(c, m, n, i, j) + alpha * accum; \
	} \
}
	
void sgemm(char atrans, char btrans, int m, int n, int k, float alpha, float const *a, float const *b, float beta, float *c)
{
	if (atrans == 'n' && btrans == 'n') {
		SGEMM_LOOP(IND, IND);
	}
	if (atrans == 'n' && btrans == 't') {
		SGEMM_LOOP(IND, INDT);
	}
	if (atrans == 't' && btrans == 'n') {
		SGEMM_LOOP(INDT, IND);
	}
	if (atrans == 't' && btrans == 't') {
		SGEMM_LOOP(INDT, INDT);
	}
}

#define SGEMM2D(at, bt, m, n, k, alpha, a, b, beta, c) sgemm(at, bt, m, n, k, alpha, AS_1D(a), AS_1D(b), beta, AS_1D(c))
	
void eyeN(float *a, int n)
{
	fzero(a, n * n);
	for (int i = 0; i < n; ++i) {
		a[n * i + i] = 1.0f;
	}
}

void zeromat(float *a, int m, int n)
{
	for (int i = 0; i < m * n; ++i) {
		a[i] = 0;
	}
}

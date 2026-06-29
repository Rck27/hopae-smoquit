#pragma once
#include <math.h>

// ── Konstanta venturi (sesuaikan dengan desain mekanik kamu) ──
#define VENTURI_D_INTAKE  0.015f     // m
#define VENTURI_D_NECK     0.0075f   // m
#define VENTURI_Cd          0.98f     // koefisien discharge
#define AIR_DENSITY         1.204f    // kg/m³
#define AIR_VISCOSITY       1.825e-5f // Pa·s

// ── Struct geometri (cukup dihitung sekali) ───────────────────
struct Venturi_t {
  float A1;       // luas penampang intake (m²)
  float A2;       // luas penampang leher   (m²)
  float A_ratio;  // 1 / sqrt(1 - (A2/A1)²)
};

// ── Hasil perhitungan ─────────────────────────────────────────
struct VenturiResult_t {
  float v_neck;     // m/s
  float v_intake;   // m/s
  float flow_m3s;   // m³/s
  float flow_Lmin;  // L/min
  float reynolds;   // bilangan Reynolds di leher
};

// ── API ───────────────────────────────────────────────────────
void venturi_init(Venturi_t *v, float d_intake, float d_neck);
bool venturi_calculate(const Venturi_t *v, float dP, VenturiResult_t *out);
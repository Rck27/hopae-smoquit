#include "Venturi.h"

void venturi_init(Venturi_t *v, float d_intake, float d_neck) {
  v->A1 = M_PI * (d_intake / 2.0f) * (d_intake / 2.0f);
  v->A2 = M_PI * (d_neck   / 2.0f) * (d_neck   / 2.0f);
  float beta = v->A2 / v->A1;
  v->A_ratio = 1.0f / sqrtf(1.0f - beta * beta);
}

bool venturi_calculate(const Venturi_t *v, float dP, VenturiResult_t *out) {
  if (dP <= 0.0f) return false;

  out->v_neck = VENTURI_Cd * v->A_ratio * sqrtf(2.0f * dP / AIR_DENSITY);
  out->v_intake = out->v_neck * (v->A2 / v->A1);
  out->flow_m3s  = out->v_neck * v->A2;
  out->flow_Lmin = out->flow_m3s * 1000.0f * 60.0f;
  out->reynolds  = (AIR_DENSITY * out->v_neck * VENTURI_D_NECK) / AIR_VISCOSITY;
  return true;
}
/**
 * @file position_calc.c
 * @brief 3D position calculation using trilateration
 * 
 * Uses least-squares algorithm to calculate tag position from
 * distance measurements to multiple anchors with known positions.
 */

#include "uwb_ips.h"
#include <zephyr/logging/log.h>
#include <math.h>

LOG_MODULE_REGISTER(position_calc, LOG_LEVEL_INF);

/*============================================================================
 * Constants
 *============================================================================*/

#define MIN_ANCHORS         3
#define MAX_ITERATIONS      50
#define CONVERGENCE_THRESH  0.001f  /* 1mm convergence threshold */

/*============================================================================
 * Trilateration Algorithm
 *============================================================================*/

/**
 * @brief Solve 3D position using linearized least-squares
 * 
 * The problem is formulated as:
 *   (x - x_i)^2 + (y - y_i)^2 + (z - z_i)^2 = d_i^2
 * 
 * Subtracting equation 1 from equations 2..n gives linear equations:
 *   A * [x, y, z]^T = b
 * 
 * where A and b are formed from anchor positions and distances.
 */
int position_calculate(const twr_measurement_t *measurements,
                       uint8_t count,
                       const anchor_info_t *anchors,
                       position_3d_t *position)
{
    if (count < MIN_ANCHORS) {
        LOG_WRN("Need at least %d anchors, got %u", MIN_ANCHORS, count);
        return -EINVAL;
    }
    
    /* Find matching anchor positions for each measurement */
    float anchor_x[8], anchor_y[8], anchor_z[8], distances[8];
    uint8_t valid_count = 0;
    
    for (uint8_t i = 0; i < count && valid_count < 8; i++) {
        if (!measurements[i].valid) {
            continue;
        }
        
        /* Find anchor position */
        for (uint8_t j = 0; j < count; j++) {
            if (anchors[j].id == measurements[i].anchor_id) {
                anchor_x[valid_count] = anchors[j].position.x_mm / 1000.0f;
                anchor_y[valid_count] = anchors[j].position.y_mm / 1000.0f;
                anchor_z[valid_count] = anchors[j].position.z_mm / 1000.0f;
                distances[valid_count] = measurements[i].distance_m;
                valid_count++;
                break;
            }
        }
    }
    
    if (valid_count < MIN_ANCHORS) {
        LOG_WRN("Not enough valid measurements");
        return -EINVAL;
    }
    
    LOG_DBG("Calculating position from %u anchors", valid_count);
    
    /* Build linear system: A * x = b
     * Using first anchor as reference, form n-1 linear equations
     */
    uint8_t n = valid_count - 1;
    float A[7][3];  /* Max 7 equations, 3 unknowns */
    float b[7];
    
    float x1 = anchor_x[0];
    float y1 = anchor_y[0];
    float z1 = anchor_z[0];
    float d1 = distances[0];
    float d1_sq = d1 * d1;
    float k1 = x1*x1 + y1*y1 + z1*z1;
    
    for (uint8_t i = 0; i < n; i++) {
        float xi = anchor_x[i + 1];
        float yi = anchor_y[i + 1];
        float zi = anchor_z[i + 1];
        float di = distances[i + 1];
        float di_sq = di * di;
        float ki = xi*xi + yi*yi + zi*zi;
        
        /* 2(xi - x1)*x + 2(yi - y1)*y + 2(zi - z1)*z = d1^2 - di^2 + ki - k1 */
        A[i][0] = 2.0f * (xi - x1);
        A[i][1] = 2.0f * (yi - y1);
        A[i][2] = 2.0f * (zi - z1);
        b[i] = d1_sq - di_sq + ki - k1;
    }
    
    /* Solve using pseudo-inverse: x = (A^T * A)^-1 * A^T * b */
    
    /* Compute A^T * A (3x3 matrix) */
    float AtA[3][3] = {{0}};
    for (uint8_t i = 0; i < 3; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            for (uint8_t k = 0; k < n; k++) {
                AtA[i][j] += A[k][i] * A[k][j];
            }
        }
    }
    
    /* Compute A^T * b (3x1 vector) */
    float Atb[3] = {0};
    for (uint8_t i = 0; i < 3; i++) {
        for (uint8_t k = 0; k < n; k++) {
            Atb[i] += A[k][i] * b[k];
        }
    }
    
    /* Compute inverse of AtA using Cramer's rule (3x3) */
    float det = AtA[0][0] * (AtA[1][1] * AtA[2][2] - AtA[1][2] * AtA[2][1])
              - AtA[0][1] * (AtA[1][0] * AtA[2][2] - AtA[1][2] * AtA[2][0])
              + AtA[0][2] * (AtA[1][0] * AtA[2][1] - AtA[1][1] * AtA[2][0]);
    
    if (fabsf(det) < 1e-10f) {
        LOG_WRN("Singular matrix - anchors may be coplanar");
        return -EINVAL;
    }
    
    float inv_det = 1.0f / det;
    
    float AtA_inv[3][3];
    AtA_inv[0][0] = (AtA[1][1] * AtA[2][2] - AtA[1][2] * AtA[2][1]) * inv_det;
    AtA_inv[0][1] = (AtA[0][2] * AtA[2][1] - AtA[0][1] * AtA[2][2]) * inv_det;
    AtA_inv[0][2] = (AtA[0][1] * AtA[1][2] - AtA[0][2] * AtA[1][1]) * inv_det;
    AtA_inv[1][0] = (AtA[1][2] * AtA[2][0] - AtA[1][0] * AtA[2][2]) * inv_det;
    AtA_inv[1][1] = (AtA[0][0] * AtA[2][2] - AtA[0][2] * AtA[2][0]) * inv_det;
    AtA_inv[1][2] = (AtA[0][2] * AtA[1][0] - AtA[0][0] * AtA[1][2]) * inv_det;
    AtA_inv[2][0] = (AtA[1][0] * AtA[2][1] - AtA[1][1] * AtA[2][0]) * inv_det;
    AtA_inv[2][1] = (AtA[0][1] * AtA[2][0] - AtA[0][0] * AtA[2][1]) * inv_det;
    AtA_inv[2][2] = (AtA[0][0] * AtA[1][1] - AtA[0][1] * AtA[1][0]) * inv_det;
    
    /* Compute solution: x = AtA_inv * Atb */
    float x = AtA_inv[0][0] * Atb[0] + AtA_inv[0][1] * Atb[1] + AtA_inv[0][2] * Atb[2];
    float y = AtA_inv[1][0] * Atb[0] + AtA_inv[1][1] * Atb[1] + AtA_inv[1][2] * Atb[2];
    float z = AtA_inv[2][0] * Atb[0] + AtA_inv[2][1] * Atb[1] + AtA_inv[2][2] * Atb[2];
    
    /* Calculate quality based on residual error */
    float total_error = 0.0f;
    for (uint8_t i = 0; i < valid_count; i++) {
        float dx = x - anchor_x[i];
        float dy = y - anchor_y[i];
        float dz = z - anchor_z[i];
        float calc_dist = sqrtf(dx*dx + dy*dy + dz*dz);
        float error = fabsf(calc_dist - distances[i]);
        total_error += error;
    }
    float avg_error = total_error / valid_count;
    
    /* Quality: 1000 = perfect, 0 = 1m average error */
    uint16_t quality = (avg_error < 1.0f) ? (uint16_t)((1.0f - avg_error) * 1000) : 0;
    
    /* Store result */
    position->x_mm = (int32_t)(x * 1000.0f);
    position->y_mm = (int32_t)(y * 1000.0f);
    position->z_mm = (int32_t)(z * 1000.0f);
    position->quality = quality;
    position->timestamp_ms = k_uptime_get_32();
    
    LOG_DBG("Position: (%.3f, %.3f, %.3f) m, quality: %u, avg_error: %.3f m",
            x, y, z, quality, avg_error);
    
    return 0;
}


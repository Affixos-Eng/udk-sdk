/**
 * @file mesh_routing.c
 * @brief Mesh routing for position data forwarding to gateway
 */

#include "uwb_ips.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(mesh, LOG_LEVEL_DBG);

/*============================================================================
 * Configuration
 *============================================================================*/

#define MAX_ROUTES          8
#define ROUTE_TIMEOUT_MS    60000  /* Routes expire after 60 seconds */

/*============================================================================
 * Static Variables
 *============================================================================*/

static routing_entry_t routing_table[MAX_ROUTES];
static uint8_t num_routes = 0;
static uint16_t gateway_id = 0;
static K_MUTEX_DEFINE(route_mutex);

/*============================================================================
 * Public Functions
 *============================================================================*/

void mesh_init(void)
{
    k_mutex_lock(&route_mutex, K_FOREVER);
    
    num_routes = 0;
    gateway_id = 0;
    memset(routing_table, 0, sizeof(routing_table));
    
    k_mutex_unlock(&route_mutex);
    
    LOG_INF("Mesh routing initialized");
}

void mesh_update_route(uint16_t source_id, uint16_t gw_id, uint8_t hop_count, int8_t rssi)
{
    k_mutex_lock(&route_mutex, K_FOREVER);
    
    uint32_t now = k_uptime_get_32();
    
    /* Update gateway ID */
    if (gateway_id == 0 || gw_id == gateway_id) {
        gateway_id = gw_id;
    }
    
    /* Look for existing route */
    for (uint8_t i = 0; i < num_routes; i++) {
        if (routing_table[i].next_hop_id == source_id) {
            /* Update if better or refresh existing */
            if (hop_count + 1 <= routing_table[i].hop_count) {
                routing_table[i].hop_count = hop_count + 1;
                routing_table[i].rssi = rssi;
            }
            routing_table[i].last_update_ms = now;
            k_mutex_unlock(&route_mutex);
            
            LOG_DBG("Updated route via 0x%04X, %u hops", source_id, hop_count + 1);
            return;
        }
    }
    
    /* Add new route if space available */
    if (num_routes < MAX_ROUTES) {
        routing_table[num_routes].dest_id = gw_id;
        routing_table[num_routes].next_hop_id = source_id;
        routing_table[num_routes].hop_count = hop_count + 1;
        routing_table[num_routes].rssi = rssi;
        routing_table[num_routes].last_update_ms = now;
        num_routes++;
        
        LOG_INF("Added route via 0x%04X, %u hops to gateway 0x%04X",
                source_id, hop_count + 1, gw_id);
    }
    
    k_mutex_unlock(&route_mutex);
}

int mesh_get_next_hop(uint16_t *next_hop)
{
    k_mutex_lock(&route_mutex, K_FOREVER);
    
    uint32_t now = k_uptime_get_32();
    uint8_t best_hops = 255;
    uint16_t best_next_hop = 0;
    
    for (uint8_t i = 0; i < num_routes; i++) {
        /* Check if route is fresh */
        uint32_t age = now - routing_table[i].last_update_ms;
        if (age > ROUTE_TIMEOUT_MS) {
            continue;
        }
        
        /* Find route with fewest hops */
        if (routing_table[i].hop_count < best_hops) {
            best_hops = routing_table[i].hop_count;
            best_next_hop = routing_table[i].next_hop_id;
        }
    }
    
    k_mutex_unlock(&route_mutex);
    
    if (best_next_hop != 0) {
        *next_hop = best_next_hop;
        return 0;
    }
    
    LOG_WRN("No route to gateway");
    return -ENETUNREACH;
}


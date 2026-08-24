# Warehouse Full No Walls: UAV radio maps

This directory contains 28 GHz UAV-BS and same-height UAV-UAV radio maps for
`warehouse_full_with_industrial_ap_no_walls`.

## Common settings

- Scene: `sionna/warehouse_full_with_industrial_ap_no_walls/warehouse.xml`
- UAV heights: 0.75, 1.5, 3.0, 5.0, and 7.5 m
- Bounds: x = [-27.0, 6.0] m, y = [0.6, 30.6] m
- Receiver cell size: 0.5 m (60 x 66 cells)
- Carrier/bandwidth: 28 GHz / 100 MHz
- Ray samples per transmitter: 100,000
- Maximum path depth: 2
- LOS, specular reflection, and refraction enabled; diffraction disabled
- Antenna: 1x1 isotropic, vertical polarization
- Random seed: 42

## UAV-BS

The industrial AP phase-center position used by the existing warehouse radio
maps is (-10.0289148922, 14.8886112153, 7.2488355375) m. Downlink power is
33 dBm and UAV uplink power is 23 dBm.

`uav_bs/bs_uav_radio_maps.npz` contains path gain plus downlink and reciprocal
uplink received-power maps with shape `(5, 60, 66)`. The directory also
contains per-height and all-height PNG overviews and `radio_map_summary.json`.

## UAV-UAV

At each height, 20 transmitter locations are arranged as a 4 x 5 sampling grid
and moved locally when needed until a 0.25 m half-extent PhysX collision box is
clear. `collision_free_tx_points.json` records all final locations.

`uav_uav/uav_uav_radio_maps.npz` contains arrays with shape
`(5, 20, 60, 66)` in `(height, transmitter, receiver y, receiver x)` order.
UAV transmit power is 23 dBm. Five PNG overviews (one 4 x 5 overview per
height) and `uav_uav_radio_map_summary.json` are included.

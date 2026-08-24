# Warehouse Full with Industrial AP: UAV-to-UAV radio maps

This directory contains same-height UAV-to-UAV radio maps for the compact
`warehouse_full_with_industrial_ap` Sionna scene.

## Simulation settings

- Carrier frequency: 28 GHz
- UAV transmit power: 23 dBm
- Bandwidth: 100 MHz
- Antenna used for geometry radio maps: 1x1 isotropic, vertical polarization
- Ray-tracing depth: 2
- Samples per transmitter map: 100,000
- Diffraction: disabled
- Specular reflection and refraction: enabled
- Receiver grid: 0.5 m, x = [-27.0, 6.0] m, y = [0.6, 30.6] m
- UAV heights: 0.75, 1.5, 3.0, 5.0, and 7.5 m
- Transmitter samples: 25 collision-free positions per height
- Random seed: 42

Each PNG is a 5x5 overview containing all 25 transmitter maps at one height.
The white star is the transmitting UAV, while each pixel is the received power
at a possible receiving-UAV position at the same height. All figures use the
same color scale, -180 to -40 dBm, so they can be compared directly.

## Outputs

- `scene_top_view_radio_map_bounds.png`: clean orthographic geometry overview
- `scene_top_view_sampling_points_all_heights.png`: scene plus the actual P01-P25
  transmitter positions at every sampled height
- `uav_uav_25points_z_0p75m.png`
- `uav_uav_25points_z_1p5m.png`
- `uav_uav_25points_z_3m.png`
- `uav_uav_25points_z_5m.png`
- `uav_uav_25points_z_7p5m.png`
- `uav_uav_radio_maps.npz`: numerical path-gain and received-power arrays
- `uav_uav_radio_map_summary.json`: simulation metadata and per-map statistics
- `collision_free_tx_points.json`: anchors and PhysX-validated transmitter points

The numerical arrays have shape `(5, 25, 60, 66)` in the order
`(height, transmitter point, receiver y, receiver x)`.

## Average receiver-grid reachability

| UAV height | Pixels above -100 dBm | Pixels above -120 dBm |
|---:|---:|---:|
| 0.75 m | 13.34% | 13.57% |
| 1.5 m | 15.64% | 15.83% |
| 3.0 m | 17.56% | 17.99% |
| 5.0 m | 23.76% | 24.61% |
| 7.5 m | 25.77% | 25.97% |

Dark-purple cells are below the displayed -180 dBm floor or not reached by the
configured ray-tracing paths; they should not be interpreted as occupied-space
masks.

The top views are projected directly from the same Sionna PLY meshes used by
the ray tracer, with the exact radio-map XY limits. The industrial AP marker is
included as a scene reference but was not an active transmitter in these
UAV-to-UAV maps.

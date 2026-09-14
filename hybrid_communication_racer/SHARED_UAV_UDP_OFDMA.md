# Distributed UAV UDP + shared OFDMA model

## Scope

The shared resource model is enabled only when communication is non-ideal and
`network_topology=distributed`. Perfect communication, AP-assisted routing,
BS round-robin, UAV-BS queues, and all BS scheduling/resource code continue to
use their existing paths.

ROS 2/DDS remains the local simulator ingress/egress bridge. Once a serialized
RACER message enters the communication proxy, its simulated air-interface
transport unit is an unacknowledged UDP datagram with an eight-byte UDP header.
No kernel TCP connection or radio-layer UDP retransmission is used. A failed
receiver can obtain the information later only if RACER's existing
bookkeeping/incremental-map logic emits another datagram.

## Data path

1. RACER emits state, trajectory, pair optimization, map chunk, chunk stamp,
   or assignment data at its original asynchronous rate.
2. The launch remapping sends the serialized message to the communication
   proxy's sender-specific ingress topic.
3. The proxy adds the UDP header and determines whether the datagram is a
   directed unicast or a multicast.
4. Exactly one datagram is inserted into that UAV's sender queue.
5. The global 0.125 ms slot scheduler collects non-empty sender queues and
   allocates disjoint shares of the 66-PRB network pool.
6. The MCS-20 CP-OFDM rate is computed from the PRBs actually allocated in
   that slot. Remaining bits stay at the queue head for later slots.
7. A multicast consumes one physical transmission and one 20 dBm UAV power
   budget. Sionna still returns one channel observation for each intended
   receiver; those observations are measurements of the same transmitted
   waveform, not separate transmissions.
8. At physical completion, each receiver independently draws success/failure
   from its accumulated SNR and UDP packet PER. Successful datagrams are
   republished to that receiver's RACER input topic. Failed datagrams vanish.

## PRB allocation

Active senders are sorted and rotated deterministically. With `N <= 66`, each
gets `floor(66/N)` PRBs and the rotating first `66 mod N` senders get one
extra PRB. PRB ranges are contiguous and non-overlapping. The rotation moves
the extra PRBs to different senders in later slots.

For UAV0, UAV1, and UAV2 active in the same slot:

| Sender | PRB range | PRBs | Approximate MCS-20 payload/slot |
|---|---:|---:|---:|
| UAV0 | 0-21 | 22 | 1,258.6 bytes |
| UAV1 | 22-43 | 22 | 1,258.6 bytes |
| UAV2 | 44-65 | 22 | 1,258.6 bytes |

The three allocations total exactly 66 PRBs. Because the radio is
half-duplex, UAV0/UAV1/UAV2 cannot receive each other's multicast during that
slot. Other UAVs can independently decode all three transmissions. A receiver
that misses any part of a multi-slot datagram because it transmits in an
overlapping slot fails that datagram with `half_duplex_blocked`.

## Power and SNR

Sionna reports full-channel SNR from one 20 dBm sender power budget. When a
sender receives fewer than 66 PRBs, the model concentrates that same total
power over its assigned PRBs:

`allocated_snr_db = full_band_snr_db + 10 log10(66 / allocated_prbs)`

The power is applied once per physical datagram, regardless of receiver count.
The PRB count affects the bit rate, while the receiver's own Sionna path gain
and allocated-band SNR affect TBLER/PER.

## Diagnostic logs and statistics

`RACER_UAV_OFDMA_SLOT` records:

`slot, sender, udp_type, packet_id, packet_size, prb_start, allocated_prbs, remaining_bytes, receivers, tx_power_dbm`

`RACER_UAV_OFDMA_RX` records:

`slot, sender, udp_type, packet_id, packet_size, allocated_prbs, remaining_bytes, receiver, snr_db, per, result`

Logging is sampled by `uav_ofdma_log_packet_stride` to keep long experiments
manageable. Set it to `1` for exhaustive packet logs. Statistics separately
record physical transmissions, power applications, receiver attempts/results,
half-duplex failures, active slots, PRB allocation events, and the maximum PRB
total observed in any slot.

## Verification

The integration test with 10 UAVs sends one UAV0 multicast to UAV1-UAV9 and
asserts all of the following:

- one UDP datagram;
- one physical transmission;
- one 66-PRB allocation for the small test packet;
- one 20 dBm power application;
- nine receiver attempts;
- five high-SNR receiver successes and four low-SNR PER failures.

Separate tests verify 22/22/22 allocation for three active senders, rotating
6/7-PRB allocation for ten active senders, PRB-limited MCS-20 rate, and
half-duplex behavior. Perfect, AP-assisted, and BS round-robin regression tests
remain in the full test suite.

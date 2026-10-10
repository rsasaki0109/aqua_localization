# Independent underwater truth discovery (2026-10-10)

This metadata-only search acquired no new sensor sequence or reference coordinates.
The provenance receipts, pinned revisions and ten candidates are recorded in
[hardware_truth_discovery.json](hardware_truth_discovery.json). It is not an
exhaustive claim about dataset availability.

| Candidate | Reference and actual access outcome |
|---|---|
| NTNU original subset3 | Publisher motion capture; three additional official ground-truth share metadata requests returned HTTP404. |
| NTNU HF / Voxel51 mirror | ReAqROVIO estimates, including converted trajectories; not motion capture. |
| AFRL stereo-VI / Pamir | COLMAP or SVIn2 trajectories; independent hardware pose truth not established. |
| OsloMet event dataset | DVL reference could support velocity validation; accumulated navigation is not externally measured drift-free full pose truth. |
| Ship-Hull Vinyl | Manual controlled motion; no external 6DOF trajectory documented. |
| York-Tank | Only preview public; hardware provenance unestablished. No application or contact. |
| AquaTriV | Publisher describes indoor motion capture, but pinned project download buttons state “Will be released soon.” |
| HAUD | A secondary AquaTriV comparison table identifies motion capture, stereo and IMU. Official page access blocked by proxy CONNECT403, with no HTTP response from publisher. Pressure availability and official provenance unconfirmed. |

The VSLAM-LAB Ariel adapter renames a ReAqROVIO baseline trajectory to
`groundtruth.csv`; renaming does not establish independent hardware truth.

The concrete next access requirement is the official metadata host
`bat.sjtu.edu.cn` for <https://bat.sjtu.edu.cn/zh/haud-dataset/>. Adding this
host to an environment draft does not apply network access. After activation,
first verify official reference provenance, native sensors, calibration and
public acquisition terms. Register the new dataset input, methods and scoring
protocol and freeze all trials before retrieving or reading reference values.
No Tank request or outreach is authorized or needed by this discovery.

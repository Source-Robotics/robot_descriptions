# PAROL6 inheritance and normalized-frame validation

Local physical properties are inherited from PAR6 by instruction. Public arm frames and the 177 mm forearm match PAR6. PAROL6 flange reach is 50.5 mm versus PAR6 57.1 mm, matching the shorter housing.

| Profile | Canonical-chain residual (mm) | Native/URDF TCP (mm) | M difference versus PAR6 (relative) | Gravity difference versus PAR6 (Nm) |
| --- | ---: | ---: | ---: | ---: |
| flange | 2.07e-12 | 0.002251 | 0.00356265 | 0.0035934 |
| msg | 5.05e-12 | 0.001000 | 0.0126109 | 0.0246035 |
| msg_tpu_glue | 5.05e-12 | 0.001000 | 0.0126109 | 0.0246035 |
| ssg48 | 0 | 0.000787 | 0.0145339 | 0.0284882 |
| ssg48_stock | 0 | 0.000787 | 0.0145339 | 0.0284882 |
| pneumatic | 2.07e-12 | 0.002474 | 0.0125118 | 0.0126255 |

Each profile uses 160 configurations. Generalized dynamics differences include the 6.6 mm shorter tool lever arm. Local mass/COM/tensors and gains remain inherited estimates.

See `KINEMATICS_COMPARISON.md`, `DYNAMICS.md`, and `ACTUATION.md` for assumptions and calibration status.

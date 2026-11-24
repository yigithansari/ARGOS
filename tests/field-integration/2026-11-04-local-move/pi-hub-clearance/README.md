# Shaft-mounted hub clearance check — 2026-11-04

Operator deferred the proposed wheel swap and reported shifting a shaft-mounted
part outward to avoid contact, then explicitly requested retest. Existing raised,
secured setup retained. Operator subsequently confirmed the left wheel was still
REMOVED during this run. Do not label it a loaded-wheel fix.

Unchanged baseline firmware CRC0C70506B,35 RPM/20 s. Final4 s:

- Left mean34.754 RPM, range33.10–36.19;90.9% of samples within±5%.
- Right mean35.107 RPM, range34.06–36.01;100% within±5%.
- Both mean criteria pass; strict instantaneous criterion fails on one left sample.
- Final zero-output STOP confirmed. No PI gain or firmware change.

Filtered RPM versus encoder phase over STM-timed8+ s: left angular harmonic
amplitude0.409 RPM, R²0.061 (original wheel-fitted baseline5.096 RPM,R²0.984).
Large periodic ripple remained absent in this wheel-removed configuration.

## Original left wheel refitted, no swap

Operator then refitted the original left wheel and explicitly reported ready in
the secured/wheels-clear setup. Same firmware,35 RPM/20 s:

- Final4 s means L35.076/R35.123 RPM; ranges L30.74–40.81/R33.99–36.09.
- Within±5% samples L27.3%/R100%; mean pass, instantaneous left failure.
- Final zero-output STOP confirmed.
- Left angular harmonic amplitude4.923 RPM,R²0.988 (STM-timed8+ s analysis).

The large once-per-revolution ripple returned on refitting the wheel despite
the hub-clearance adjustment. Removal/refit evidence implicates a wheel-mounted
load/mount interaction, without yet distinguishing wheel defect, coupling or
motor shaft/bearing under load. No gains/source changed. CRC remains0C70506B.

## Outer rings exchanged; hubs stayed on original sides

Operator clarified each wheel consists of a hub and an outer ring. Only outer
rings were exchanged; original hubs stayed on their motor shafts. This is NOT
a full wheel/hub swap. Same secured/wheels-clear35 RPM/20 s test:

- Final4 s means L35.609/R34.725 RPM; ranges L31.16–41.09/R33.90–35.28.
- Within±5% samples L18.2%/R100%; mean pass, left instantaneous failure.
- Final zero-output STOP confirmed.
- Angular harmonic amplitude L4.994/R0.170 RPM; left R²0.984 (STM8+ s).

Ripple stayed on the LEFT and did not follow the exchanged outer rings. The
original left hub, hub-to-shaft interface and loaded motor/shaft behavior remain
candidates. The ring exchange does not establish any one cause or prove all
outer-ring effects absent.

Next requested: power off and exchange complete HUB+RING assemblies, so the
original left HUB moves onto the right motor shaft. Preserve electrical wiring;
secure both wheels clear and report ready. Do not force an unremovable hub.
Hubs not yet confirmed exchanged; no further motor job scheduled. Last verified
state STOP, original hub sides, exchanged outer-ring sides. Firmware unchanged.

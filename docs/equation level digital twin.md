# Equation-level literature review for a CNC milling digital twin of titanium and exotic alloys

A practical, high-fidelity digital twin for CNC end-milling of Ti-6Al-4V and nickel superalloys requires roughly **40–60 governing equations** spanning nine coupled modeling domains — from chip geometry through cybersecurity. The minimum viable real-time stack reduces to approximately 15 core equations executing at servo-loop rates (1–4 kHz), with heavier thermo-mechanical and wear models running at near-real-time (10–500 ms) or offline timescales. This review assembles the complete modeling stack from primary sources, extracting every governing equation, its calibrated parameters for hard alloys, validity bounds, and implementation pathway for a production digital twin.

The architecture that emerges from the literature follows a three-tier computational budget: **real-time** (chip thickness, mechanistic forces, stability check), **near-real-time** (wear ODE integration, thermal field via POD, coefficient updates via Kalman filter), and **offline** (FEA calibration, J-C parameter fitting, FRF measurement, CWE map pre-computation). Ward et al. (2021) and Bakhshandeh et al. (2024) from the Altintas group have demonstrated this architecture on industrial CNC cells, with CWE lookup + mechanistic forces running in sub-millisecond cycles.

---

## Category 1: Constitutive models and serrated chip physics for titanium and superalloys

### The Johnson-Cook constitutive equation and its calibrated parameters

The standard Johnson-Cook flow stress model remains the foundation for all FEA-based chip formation simulations:

**σ = [A + Bεⁿ] · [1 + C·ln(ε̇/ε̇₀)] · [1 − ((T−T_r)/(T_m−T_r))^m]**

where A is initial yield strength (MPa), B is strain hardening modulus, n is strain hardening exponent, C is strain-rate sensitivity, m is thermal softening exponent, ε̇₀ is the reference strain rate (typically 1 s⁻¹), T_r is room temperature, and T_m is melting temperature.

For **Ti-6Al-4V**, three widely used calibration sets exist. The Lee & Lin (1998) set — **A = 782.7, B = 498.4, n = 0.28, C = 0.028, m = 1.0** — is the most validated for machining FEA and is recommended as the baseline. The Meyer & Kleponis (2001) set (A = 862.5, B = 331.2, n = 0.34, C = 0.012, m = 0.8) covers strain rates up to 2150 s⁻¹. The Kay (2003) DOT/FAA set (A = 1098, B = 1092, n = 0.93, C = 0.014, m = 1.1) captures higher strains but can overestimate forces. Sima & Özel (2010) demonstrated that Lee-Lin parameters produce the closest match to experimental cutting forces and chip morphology for Ti-6Al-4V at V_c = 120 m/min.

For **Inconel 718**, commonly used parameters are **A = 1241, B = 622, n = 0.6522, C = 0.0134, m = 1.3** with T_m = 1297°C. Iturbe et al. calibrated via split-Hopkinson bar at 5000–11000 s⁻¹: A = 1200, B = 1284, n = 0.54, C = 0.006, m = 1.2. For Waspaloy, RR1000, and René 41, open-literature J-C data is extremely limited; inverse identification from cutting experiments or adaptation from Inconel 718 is recommended.

### Modified J-C models that produce serrated chips without explicit damage

Standard J-C cannot produce serrated chips in FEA without a separate damage criterion. The **Calamaz TANH model** (2008) introduces strain softening via a hyperbolic tangent modifier:

**σ = [A + Bεⁿ·(1/exp(εᵃ))] · [1 + C·ln(ε̇/ε̇₀)] · [1 − T*ᵐ] · [D + (1−D)·tanh(1/(ε+p)ʳ)ˢ]**

where D = 1−(T/T_m)^d, and parameters a, p, r, s control softening onset and depth. Sima & Özel (2010, Int J Mach Tools Manuf 50:943–960) tested three variants and found **Model 3** (s = 0.05, a = 2, r = 2, d = 1) produced the best match to experimental serrated chip morphology for Ti-6Al-4V. This model produces serrated chips purely through adiabatic shearing without requiring a separate damage criterion, valid for cutting speeds 30–300 m/min, strain rates 10³–10⁶ s⁻¹, and temperatures up to 1200°C.

### Johnson-Cook damage model

The fracture strain is:

**ε_f = [D₁ + D₂·exp(D₃·η)] · [1 + D₄·ln(ε̇/ε̇₀)] · [1 + D₅·T*]**

where η = σ_m/σ̄ is stress triaxiality. Cumulative damage follows **D = Σ(Δε/ε_f)**, with element failure at D = 1 (or D_cr = 0.8–1.0). For Ti-6Al-4V, the Kay (2003) parameters — **D₁ = −0.09, D₂ = 0.25, D₃ = −0.5, D₄ = 0.014, D₅ = 3.87** — are the most commonly used in machining simulation. For Inconel 718, approximate values are D₁ = 0.04, D₂ = 0.75, D₃ = −1.45, D₄ = 0.04, D₅ = 0.89, though these vary significantly across studies.

### Adiabatic shear band criteria and chip segmentation

The Zener-Hollomon thermoplastic instability criterion states that adiabatic shear instability occurs when thermal softening overcomes strain hardening: **(∂τ/∂γ)|_T + (∂τ/∂T)|_γ · (dT/dγ) ≤ 0**. Recht's (1964) critical strain rate for ASB initiation is **γ̇_c ≥ [ρ·C_v·k·(∂τ/∂γ)] / [h²·(−∂τ/∂T)]**, where h is shear band thickness. Ti-6Al-4V is approximately **1400× more susceptible** to adiabatic shear banding than medium carbon steel, meaning serrated chips form at virtually all conventional cutting speeds (V_crit ≈ 30 m/min). The chip segmentation frequency follows **f_seg = V_c / L_seg**, and the degree of segmentation is **G_s = (h_max − h_min)/h_max**, ranging from 0.3–0.8 for Ti-6Al-4V depending on speed.

### Tool-chip interface friction

Zorev's (1963) split sticking/sliding model defines **τ_f = min(μ·σ_n, τ_Y)**, where τ_Y = σ_Y/√3 is the shear yield stress. Özel & Sima (2010) refined this into a three-region model for Ti-6Al-4V: full sticking (m = 1) near the tool tip, constant shear friction (m = 0.70–0.85) on the intermediate rake face, and Coulomb friction (μ = 0.5) beyond the chip contact zone. For Ti-6Al-4V, the average apparent friction coefficient ranges from **μ = 0.3–0.6**, decreasing with cutting speed due to thermal softening. Sticking dominates in Ti alloy cutting.

### Heat partition at the tool-chip interface

The Komanduri-Hou (2000/2001) three-part series provides analytical solutions using modified Jaeger moving-heat-source integrals. The heat partition ratio at the shear plane follows the simplified Loewen-Shaw form: **R_w ≈ 1 / (1 + 0.754·√(N_th/tan φ))**, where N_th = V·t₁/(4α_w) is the thermal number. For Ti-6Al-4V, approximately **75–80% of total heat** goes into the chip, 10–15% into the tool, and only 5–10% into the workpiece (due to low thermal conductivity of ~6.7 W/m·K at room temperature, rising to ~18 at 800°C).

---

## Category 2: Cutter-workpiece engagement geometry and instantaneous chip thickness

### Helical end mill chip thickness — the foundational geometric equation

For a helical end mill with N teeth, helix angle β, and diameter D, the immersion angle of tooth j at axial height z is:

**φ_j(z) = φ + j·(2π/N) − (2 tan β / D)·z**

where the helix lag parameter **k_β = 2 tan β / D** [rad/mm] governs how much the engagement angle shifts per unit height. The classical instantaneous uncut chip thickness under the circular-path approximation (valid when f_t ≪ R) is:

**h(φ_j) = f_t · sin(φ_j(z))**

valid only when φ_st ≤ φ_j(z) ≤ φ_ex, where f_t is feed per tooth. For **up milling**: φ_st = 0, φ_ex = arccos(1 − a_e/R). For **down milling**: φ_st = arccos(a_e/R), φ_ex = π. The maximum helix lag angle is **ψ = k_β · a_p**. This equation, combined with the axial discretization into M differential disk elements of thickness dz = a_p/M, forms the geometric backbone of every mechanistic force model.

### Dynamic chip thickness with regenerative vibration

When structural vibrations are included (Montgomery & Altintas, 1991), the chip thickness becomes:

**h_j(φ_j) = [f_t·sin φ_j + (x_j − x_{j−1})·sin φ_j + (y_j − y_{j−1})·cos φ_j] · g(φ_j)**

where x_{j−1}, y_{j−1} are vibration displacements at the previous tooth passage (regenerative delay T = 60/(N·n)), and g(φ_j) is the engagement window function. This expression creates the delay-differential equation governing regenerative chatter.

### Runout model

The two-parameter runout model (Kline & DeVor, 1983) modifies chip thickness to:

**h_j(φ_j, z) = f_t·sin φ_j + r₀·sin(φ_j − γ₀) − r₀·sin(φ_{j−1} − γ₀)**

where r₀ is the radial eccentricity offset and γ₀ is the locating angle. This causes uneven chip loads between teeth; when h < 0, the tooth loses contact.

### CWE computation methods for complex toolpaths

For simple 3-axis operations, analytical φ_st/φ_ex formulas suffice. For 5-axis, trochoidal, or sculptured-surface milling, four computational methods exist in order of increasing accuracy and cost: **(1) Analytical** (Hendriko et al., 2014) using grazing-point extension from swept-envelope theory; **(2) Z-map** (Sun et al., 2009) discretizing the workpiece into a 2D height grid; **(3) Dexel/voxel** (UBC MAL group) using ray-intersection or voxel-tracing algorithms; and **(4) Solid modeler/B-rep** (Aras et al., 2014) using Boolean subtraction. Ward et al. (2021) demonstrated that pre-computing CWE maps offline and indexing by CL position enables real-time lookup via Euclidean distance matching — the key enabler for production-speed digital twins.

For **ball-end mills**, the effective radius varies with height as R(z) = √(R₀² − (R₀−z)²), and chip thickness becomes **h(ψ, φ) = f_t · sin φ · sin ψ**, where ψ is the zenith angle. Engin & Altintas (2001) defined a generalized 7-parameter cutter envelope covering cylindrical, ball, tapered, and bull-nose geometries.

---

## Category 3: Mechanistic force model equations and coefficient identification

### The Altintas mechanistic cutting force model

The differential forces on a cutting edge element at height dz decompose into shearing and ploughing components (Altintas & Lee, 1996):

**dF_t = [K_tc · h(φ_j) + K_te] · dz**
**dF_r = [K_rc · h(φ_j) + K_re] · dz**
**dF_a = [K_ac · h(φ_j) + K_ae] · dz**

where K_tc, K_rc, K_ac are cutting coefficients [N/mm²] and K_te, K_re, K_ae are edge coefficients [N/mm]. The transformation to Cartesian forces uses the rotation matrix: **dF_x = −dF_t cos φ − dF_r sin φ**, **dF_y = +dF_t sin φ − dF_r cos φ**, **dF_z = dF_a**. Total forces are obtained by summing over all engaged teeth and all axial disk elements.

For **Ti-6Al-4V** with carbide tooling, typical values are: K_tc ≈ 1800–2400 N/mm², K_rc ≈ 600–1000 N/mm², K_ac ≈ 300–700 N/mm², K_te ≈ 10–30 N/mm, K_re ≈ 15–40 N/mm, K_ae ≈ 5–15 N/mm. For **Inconel 718**: K_tc ≈ 2000–3200 N/mm², K_rc ≈ 800–1500 N/mm², with significant variation based on cooling strategy (values drop under emulsion vs. dry/MQL).

### Orthogonal-to-oblique transformation

Budak, Altintas & Armarego (1996) derived the oblique cutting coefficients analytically from orthogonal cutting database parameters (τ_s, φ_c, β_a):

**K_tc = τ_s · [cos(β_n − α_n) + tan η_c · tan i · sin β_n] / [sin φ_n · √(cos²(φ_n + β_n − α_n) + tan² η_c · sin² β_n)]**

with analogous expressions for K_rc and K_ac. This eliminates recalibration for each tool geometry — once the orthogonal database is established for a material, coefficients for any oblique geometry are predicted analytically.

### Average force method for calibration

The average force per tooth period is linear in feed per tooth: **F̄_q = A_q · f_z + B_q** (q = x,y,z), where slopes A_q contain cutting coefficients and intercepts B_q contain edge coefficients. Performing slot milling tests at 3–5 feed rates and applying linear regression yields all six coefficients. The Kienzle specific cutting force model **k_c = k_c1.1 · h^(−m_c)** provides an alternative single-equation approach, with k_c1.1 ≈ 1350–1720 N/mm² and m_c ≈ 0.22–0.30 for Ti-6Al-4V.

### Online coefficient identification for digital twins

Grossi et al. (2020) demonstrated two methods for continuous in-process coefficient updating: **(1) Recursive Least Squares (RLS)**, which updates coefficient estimates with each new force measurement, and **(2) Ensemble Kalman Filter (EnKF)**, which tracks coefficient uncertainty through an ensemble of particles. The EnKF shows extraordinary robustness against measurement noise. Grossi (2017) also showed that tangential coefficients can be identified purely from spindle power signals: **K_tc = (P_spindle − P_idle) / MRR**, eliminating the need for external dynamometers. The general recursive update form is **K̂(k+1) = K̂(k) + G(k)·[F_meas − F_pred]**, where G is the Kalman gain.

---

## Category 4: Regenerative chatter stability via delay differential equations

### The governing DDE for milling dynamics

The regenerative milling dynamics are governed by a delay-differential equation. The cutting force matrix at any instant is:

**{F(t)} = ½ · a_p · K_t · [A(t)] · {q(t−T) − q(t)}**

where **T = 60/(N·n)** is the tooth-passing period, [A(t)] is the time-varying directional coefficient matrix with elements α_xx = Σ −g_j[sin 2φ_j + K_r(1−cos 2φ_j)], and analogous terms for α_xy, α_yx, α_yy. The full 2-DOF DDE is:

**M q̈(t) + C q̇(t) + K q(t) = ½ a_p K_t [A(t)] {q(t−T) − q(t)}**

### Altintas-Budak zero-order approximation for stability lobes

The zero-order approximation (Altintas & Budak, 1995) averages the directional coefficients over one tooth period:

**[A₀] = (N/2π) · ∫_{φ_st}^{φ_ex} [A(φ)] dφ**

yielding closed-form integrals (e.g., α_xx = ½[cos 2φ − 2K_r φ + K_r sin 2φ] evaluated at limits). The stability eigenvalue problem is:

**det([I] + Λ · [A₀] · [Φ(iω_c)]) = 0**

where **Λ = −(N/4π) · a_p · K_t · (1 − e^{−iω_c T})** and [Φ(iω_c)] is the FRF matrix at chatter frequency ω_c. This yields the critical depth of cut **a_lim = −2πΛ_R(1+κ²) / (N·K_t)** where κ = Λ_I/Λ_R, and spindle speed **n = 60ω_c / [N(π − 2 arctan κ + 2kπ)]** for k = 0, 1, 2, … The ZOA computes in milliseconds but is valid only above ~25% radial immersion.

### Semi-discretization method for arbitrary conditions

Insperger & Stépán (2002, 2004) developed the semi-discretization method (SDM) for arbitrary immersion, variable pitch, helix effects, and nonlinear dynamics. The DDE is rewritten in first-order form **q̇(t) = [L(t)]q(t) + [R(t)]q(t−T)**, then the tooth period T is divided into m intervals (Δt = T/m). The solution at each step uses matrix exponentials:

**q_{i+1} = e^{[L_i]Δt} · q_i + (e^{[L_i]Δt} − I) · [L_i]⁻¹ · [R_i] · (q_{i−m+1} + q_{i−m})/2**

The **Floquet transition matrix** Φ = B_m · B_{m−1} ··· B_1 over one tooth period determines stability: **max|eigenvalue(Φ)| < 1 → stable**. Period-doubling bifurcation occurs when μ = −1; quasi-periodic chatter when μ = e^{±iω_c}. Typically m ≈ 40 suffices; SDM handles variable pitch, helix, and variable speed natively.

### Process damping at low cutting speeds

Process damping arises from tool flank-workpiece interference. Budak & Tunc (2010, 2013) model the damping force as **F_pd = K_sp · a_p · A_ind**, where A_ind is the indentation area between the tool's clearance face and the wavy machined surface. For sinusoidal vibration of amplitude R at frequency ω_c: **A_ind ≈ R²·ω_c / (2·V_c·tan γ)**. This effect is critical for titanium and nickel alloy machining: process damping dramatically increases stability at low cutting speeds where many vibration waves fit per revolution.

---

## Category 5: Thermo-mechanical coupling, temperature fields, and surface integrity

### Heat generation in three deformation zones

Total heat generation follows **Q_total = F_c · V_c**, with approximately **90% of mechanical work** converted to heat (Taylor-Quinney coefficient β = 0.9). Heat sources decompose into primary shear zone (**Q_s = F_s · V_s**), secondary rake-face friction (**Q_f = F_f · V_chip**), and tertiary flank-face rubbing.

The shear plane temperature rises by: **ΔT_shear = (η · F_s · V_s) / (ρ · c_p · A_s · V)**, where A_s = (t₁·w)/sin φ is the shear plane area. The Komanduri-Hou (2000/2001) three-part series provides the most complete analytical framework, modeling the shear plane as a moving oblique band heat source:

**ΔT_M = (q_s / 2πk) · ∫₀ˡ exp[−V(X−x')/(2α)] · K₀[V·r_M/(2α)] · dl'**

where K₀ is the modified Bessel function of the second kind. The heat partition ratio emerges from matching temperature continuity across the shear plane rather than being prescribed a priori.

### Transient 3D heat conduction

The governing PDE for the workpiece thermal field is:

**ρc_p · ∂T/∂t = k·∇²T + q̇_gen**

Lazoglu & Altintas (2002) solved this via finite differences for interrupted milling, with stability criterion Δt ≤ Δx²/(4α). For a **50×50 grid**, each time step requires ~10,000 multiply-add operations — achievable at **>10 kHz** on modern processors, making FDM highly suitable for real-time digital twin thermal modeling.

### Residual stress prediction

Residual stresses arise from the superposition of mechanical (compressive, from ploughing) and thermal (tensile, from rapid heating-cooling cycles) contributions. For Ti-6Al-4V: low cutting speeds produce compressive residual stresses (mechanical dominant), while high speeds increase tensile stresses. Wang et al. (2022) achieved 11.6–15.2% average prediction error using coupled FEM with J-C constitutive models and C3D8RT elements.

### Thermal error compensation for CNC machines

The volumetric thermal error model takes the form **E(x,y,z,T) = E_geo(x,y,z) + Σ_j Σ_i c_ij · f_j(x,y,z) · ΔT_i**, where f_j are polynomial basis functions and c_ij are sensitivity coefficients. Lu et al. (2023) demonstrated a DT-LSTM hybrid achieving **>98% accuracy** for spindle thermal error prediction, with thermal error reductions of **75–85%** using only 7 temperature sensors.

---

## Category 6: Tool wear rate equations coupled to force and temperature

### The Usui wear rate equation

The most widely used wear model for FEA-coupled simulation is:

**dW/dt = B₁ · σ_n · V_s · exp(−B₂/T)**

where σ_n is normal contact pressure, V_s is sliding velocity, T is interface temperature, and B₁, B₂ are calibration constants. The equation represents thermally activated adhesive wear: the product σ_n·V_s captures mechanical intensity while the Arrhenius exponential captures thermal activation. **Critical finding**: assuming V_s = V_c leads to poor predictions at low speeds; V_s must be computed from FE contact analysis (Hosseinkhani & Ng, 2020). Calibration requires FE simulations at 5–6 wear states plus ≥2 cutting experiments.

### The Takeyama-Murata model separating wear mechanisms

**dVB/dt = C₁·V·exp(−C₂/T) + D·exp(−E/(R·T))**

The first term captures abrasive/mechanical wear and the second captures diffusion wear. When temperature exceeds **700–800°C** — routinely reached in Ti-6Al-4V and Inconel 718 machining — the abrasive term can be neglected and diffusion dominates. Attanasio et al. (2008, 2010) implemented this in 3D FE with temperature-dependent diffusion coefficients.

### Diffusion wear mechanism specific to Ti-6Al-4V + WC-Co

Multiple TEM/SEM studies reveal a multi-step degradation process: outward carbon diffusion from WC grains forms metallic tungsten, which dissolves into Ti causing β-phase transformation, while inward Ti diffusion forms TiC. The thermodynamic driving force (WC + Ti → TiC + W) has **~10× larger Gibbs free energy** than the analogous reaction with iron, explaining why Ti alloys are far more aggressive to carbide tooling than steels.

### Pálmai's unified nonlinear ODE

Pálmai (2013) unified the wear stages into a single autonomous ODE:

**dVB/dt = a₁·exp(−b₁/T(VB)) + a₂·VB·exp(−b₂/T(VB))**

where T is itself a function of VB (via rubbing-heat feedback), creating a nonlinear system that naturally captures all three wear stages: break-in, steady-state, and accelerating failure.

### Wear-force coupling and state-space formulation

Flank wear modifies cutting forces through ploughing: **F_total = F_shearing + K_w · VB · w**, where K_w is the wear-land contact force intensity. This creates the state vector **x = [VB, KT, T_tool, T_workpiece]ᵀ** with coupled evolution equations. The ODE integration is computationally trivial (~microseconds per step), making real-time wear tracking feasible. The Extended Kalman Filter approach — comparing measured forces against predicted F_shear + K_w·VB·w — enables real-time VB estimation and remaining useful life prediction.

---

## Category 7: Reduced-order models and surrogates for real-time execution

### POD-Galerkin for thermal field reduction

Proper Orthogonal Decomposition decomposes the temperature field as **T(x,t) ≈ Σ_{k=1}^{L} a_k(t) · Φ_k(x)** where L ≪ M (full DOF). With just **3 POD modes**, Pulimeno et al. (2023) achieved ~1% error versus full FEM for dynamic thermal analysis — representing a **~10,000× speedup** with ~5 orders of magnitude DOF reduction. POD-RBF surrogates achieve global MRE < 3% with robust temporal extrapolation capability. For the machining digital twin, POD modes are pre-computed from offline FEA snapshots spanning the cutting parameter space, then modal coefficients a_k(t) are updated in real-time at ~10–100 ms intervals.

### Hybrid physics-AI force models with cross-material transfer

A sequential physics-informed machine learning (PIML) architecture (J. Manuf. Processes, 2024) uses the Altintas mechanistic model as Stage 1 and an ML correction layer (trained on residuals) as Stage 2. Trained on Al7075, Steel 1050, and Ti-6Al-4V, this model successfully predicted forces for **Inconel 625 without additional testing** by parameterizing materials through their thermomechanical properties. Accuracy reached **97% on training data, 94% on unseen test data**. The hybrid architecture pattern — **F_total = F_mechanistic + ΔF_ML** — captures tool deflection, material heterogeneity, coolant effects, and wear-dependent changes that pure mechanistic models miss.

### Computational budget by timescale

The recommended architecture allocates models across three tiers:

- **Real-time (< 1 ms)**: Chip thickness h = f_z sin φ (analytical), mechanistic forces via pre-computed CWE lookup (Ward et al.), stability check against pre-computed SLD, chatter detection via FFT
- **Near-real-time (10–500 ms)**: Wear ODE integration (Usui + GP correction), thermal field via POD (3–10 modes), force coefficient updates via EnKF, residual stress estimation
- **Offline (seconds to hours)**: CWE map generation via dexel/Z-buffer, 3D FEA for POD training, J-C parameter fitting, FRF measurement, GP/Kriging training from DOE

---

## Category 8: Digital twin architecture, data models, and standards

### ISO 23247 four-domain framework

ISO 23247:2021 partitions the digital twin system into four domains: **(1) Observable Manufacturing Elements** (CNC machine, tools, workpiece, sensors), **(2) Data Collection and Device Control Entity** (MTConnect adapters, OPC UA servers, MQTT brokers), **(3) Digital Twin Entity** (simulation models, data analytics, fusion, synchronization), and **(4) User Entity** (MES, ERP, PLM, custom analytics). NIST implementations have demonstrated 15–25% production cost/time reductions.

### MTConnect data items for milling

MTConnect (ANSI/MTC1.4) uses a client-server architecture with Adapter → Agent → Client. Critical data items include **SpindleSpeed** (actual RPM from encoder), **PathFeedrate** (actual mm/min), **Position** (X, Y, Z, A, B actual/commanded), **Load** (spindle/axis %), and **Execution** state. Sample rates support sub-millisecond capture. The standard is **read-only** — for closed-loop control, OPC UA or direct controller APIs are required.

### OPC UA for Machine Tools (OPC 40501-1)

The companion specification defines a MachineToolType information model with Identification, Monitoring (spindle, axis, channel), Production (job management, KPI per ISO 22400), and Equipment (tool life tracking). OPC UA provides bidirectional read/write capability with **built-in certificate-based security** — critical advantages over MTConnect for control applications.

### The Ward et al. lookahead architecture

Ward et al. (2021, Int J Adv Manuf Tech 117:3615–3629) demonstrated the production-ready architecture: CWE maps are pre-computed offline and indexed by cutter-location position. During machining, Euclidean distance matching between current TCP position (from real-time encoder data) and indexed CL points enables sub-millisecond CWE lookup. Combined with live feed rate and spindle speed, this yields real-time force prediction, enabling closed-loop MIRS (Multiple Input Reference Signal) control for autonomous machining adjustment — the first demonstrated online residual stress control.

---

## Category 9: Cybersecurity layers for CNC digital twins

### Physics-based anomaly detection using the digital twin

The digital twin serves as a physics-based baseline for cybersecurity through residual-based detection: **r(t) = y_measured(t) − y_predicted_DT(t)**. When |r(t)| > τ, an anomaly flag triggers. Balta et al. (2023, IEEE Trans. Automation Science and Engineering) demonstrated a framework that distinguishes expected anomalies (e.g., tool entry transients) from cyber-attacks using a process-of-elimination algorithm combining ML anomaly detectors, a knowledge base of known anomalies, and human-expert review.

### G-code integrity verification

Rossel et al. (USENIX Security 2025) systematically identified G-code attack vectors: firmware-based attacks, malware interception, MITM during transfer, and persistent EEPROM manipulation. Pearce et al. (2021) demonstrated detection via statistical analysis of G-code features (G0/G1 ratios, extrusion lengths, layer counts), achieving **83.3% detection rate with zero false positives**. Cryptographic signing and semantic verification of G-code against expected toolpath geometry remain emerging approaches.

### Side-channel monitoring as independent verification

The PowerGuard framework reconstructs CNC trajectories from servo motor current signals, achieving **0.047 mm trajectory reconstruction error** and **93.35% attack detection rate** across Siemens and Fanuc controllers. Multi-sensor approaches combining spindle current (250 Hz), vibration (50 kHz), cutting forces (50 kHz), and acoustic emission (1 MHz) provide an air-gapped verification channel that attackers cannot easily compromise simultaneously.

### Network security and standards compliance

Defense-in-depth requires six layers: physical access control, network segmentation (Purdue model with industrial DMZ), application security (G-code verification), process monitoring (DT-based anomaly detection), data security (encrypted storage), and organizational policies. Key standards include **NIST SP 800-82 Rev. 3** (OT security guide, September 2023), **IEC 62443** (security levels SL 1-4, zone/conduit model), OPC UA certificate-based security profiles, and zero-trust architecture with micro-segmentation isolating CNC machines into secured network zones.

---

## The minimum equation set for a practical milling digital twin

A production-ready digital twin for Ti-6Al-4V end milling requires the following irreducible set of equations executing at their respective timescales:

**Real-time geometric core (< 1 ms):** The helical immersion angle φ_j(z) = φ + j·2π/N − (2 tan β/D)·z and chip thickness h = f_t sin φ_j, evaluated at each axial disk element with engagement window g(φ_j). The three mechanistic force equations dF_{t,r,a} = [K_{tc,rc,ac}·h + K_{te,re,ae}]·dz with Cartesian rotation, summed over all engaged teeth and disks.

**Near-real-time physics (10–500 ms):** The Usui wear ODE dW/dt = B₁σ_nV_s exp(−B₂/T) integrated with force-wear coupling F_total = F_shear + K_w·VB·w. A 2D thermal FDM solver (Lazoglu-Altintas) or POD-reduced thermal field with 3–10 modes. Recursive coefficient update K̂(k+1) = K̂(k) + G(k)·[F_meas − F_pred].

**Offline calibration:** The Altintas-Budak ZOA eigenvalue problem det([I] + Λ[A₀][Φ(iω_c)]) = 0 for stability lobe diagrams, updated when FRFs or coefficients change. The J-C constitutive model calibrated via inverse FEA for chip formation validation. CWE map pre-computation indexed by CL position.

This minimum stack — roughly **15 core equations** for real-time, extended to ~40 with near-real-time and offline components — enables force prediction within 5–15% of dynamometer measurements, wear tracking with sub-100μm accuracy, chatter prediction before onset, and thermal error compensation achieving 75–85% error reduction, as demonstrated in industrial implementations by the Altintas group and the Twin-Control EU project.
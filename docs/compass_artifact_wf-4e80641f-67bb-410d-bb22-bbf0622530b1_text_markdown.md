# Governing differential equations for a CNC milling digital twin of titanium and exotic alloys

The complete dynamical system governing CNC milling of Ti-6Al-4V and nickel superalloys is a stiff, multi-timescale, coupled set of delay differential equations spanning structural vibrations (kHz), thermal fields (Hz), and wear evolution (mHz). This report presents every constitutive equation, ODE, and DDE required to build a physics-based digital twin — from the infinitesimal cutting-edge element to the full coupled state-space — with calibrated parameters for Ti-6Al-4V, Inconel 718, Waspaloy, and hardened steels. The formulations are drawn from Altintas, Oxley, Komanduri-Hou, Usui, and the Insperger-Stépán stability framework, unified into a single implementable system.

---

## 1. Flute-level instantaneous cutting force equations

The mechanistic force model discretizes each helical flute into infinitesimal axial elements dz. For a cylindrical end mill with $N_t$ teeth, radius $R$, and helix angle $\beta$, the immersion angle of tooth $j$ at axial height $z$ is

$$\phi_j(z) = \phi - (j-1)\frac{2\pi}{N_t} - \frac{z \tan\beta}{R}$$

where $\phi$ is the cutter rotation angle and the last term is the **helix lag angle** $\psi(z) = z\tan\beta / R$. The instantaneous uncut chip thickness for tooth $j$ is

$$h_j(\phi, z) = f_t \sin\bigl(\phi_j(z)\bigr) \cdot g\bigl(\phi_j(z)\bigr)$$

where $f_t$ is feed per tooth and $g(\phi_j) = 1$ when $\phi_{\text{st}} \le \phi_j \le \phi_{\text{ex}}$, zero otherwise. The engagement angles are: **up-milling** $\phi_{\text{st}} = 0$, $\phi_{\text{ex}} = \arccos(1 - a_e/R)$; **down-milling** $\phi_{\text{st}} = \arccos(a_e/R)$, $\phi_{\text{ex}} = \pi$; **slotting** $\phi_{\text{st}} = 0$, $\phi_{\text{ex}} = \pi$.

### Differential force elements on each edge segment

Each infinitesimal axial element $dz$ contributes three orthogonal force components:

$$dF_t = \bigl[K_{tc}\,h_j(\phi_j,z) + K_{te}\bigr]\,dz \qquad \text{(tangential)}$$
$$dF_r = \bigl[K_{rc}\,h_j(\phi_j,z) + K_{re}\bigr]\,dz \qquad \text{(radial)}$$
$$dF_a = \bigl[K_{ac}\,h_j(\phi_j,z) + K_{ae}\bigr]\,dz \qquad \text{(axial)}$$

Here $K_{tc}, K_{rc}, K_{ac}$ are the **cutting (shearing) force coefficients** (N/mm²) and $K_{te}, K_{re}, K_{ae}$ are the **edge (ploughing) force coefficients** (N/mm). For uncoated WC-Co tools on Ti-6Al-4V, typical ranges are $K_{tc} \approx 1800\text{–}2200$ N/mm², $K_{rc} \approx 700\text{–}1000$ N/mm², $K_{ac} \approx 400\text{–}700$ N/mm², with edge coefficients $K_{te} \approx 20\text{–}40$ N/mm, $K_{re} \approx 15\text{–}35$ N/mm, $K_{ae} \approx 5\text{–}15$ N/mm.

### Integration along the helix and coordinate transformation

The total force on tooth $j$ is obtained by integrating from the lower to upper axial engagement limits:

$$F_{q,j}(\phi) = \int_{z_{j,\text{lower}}}^{z_{j,\text{upper}}} \bigl[K_{qc}\,f_t\sin(\phi_j(z)) + K_{qe}\bigr]\,g(\phi_j(z))\,dz, \qquad q \in \{t,r,a\}$$

The integration limits are derived from the helix-engagement intersection:

$$z_{j,\text{lower}} = \max\!\Bigl\{0,\;\frac{R\bigl[\phi - (j{-}1)\tfrac{2\pi}{N_t} - \phi_{\text{ex}}\bigr]}{\tan\beta}\Bigr\}, \qquad z_{j,\text{upper}} = \min\!\Bigl\{a_p,\;\frac{R\bigl[\phi - (j{-}1)\tfrac{2\pi}{N_t} - \phi_{\text{st}}\bigr]}{\tan\beta}\Bigr\}$$

Closed-form integration is possible via the substitution $u = \phi_j(z)$, yielding $\int \sin(\phi_j)\,dz = (R/\tan\beta)\cos(\phi_j)$. The total cutting force is summed over all teeth: $F_q(\phi) = \sum_{j=0}^{N_t-1} F_{q,j}(\phi)$.

The **coordinate transformation** from tool to workpiece coordinates uses the verified Altintas convention:

$$\begin{bmatrix} F_x \\ F_y \\ F_z \end{bmatrix} = \begin{bmatrix} -\cos\phi_j & -\sin\phi_j & 0 \\ \sin\phi_j & -\cos\phi_j & 0 \\ 0 & 0 & 1 \end{bmatrix} \begin{bmatrix} dF_t \\ dF_r \\ dF_a \end{bmatrix}$$

### Cutter runout modeling

Radial runout $\rho$ at angular location $\lambda$ modifies the effective radius of tooth $j$ to $R_j = R + \rho\sin\bigl((j{-}1)\tfrac{2\pi}{N_t} + \lambda\bigr)$. The chip thickness becomes:

$$h_j(\phi_j) = f_t\sin(\phi_j) + \Delta r_j$$

where $\Delta r_j = R_j - R_{j-1}$. For a 4-flute cutter, $\Delta r_j = 2\rho\sin(\pi/N_t)\cos\bigl((j{-}1)\tfrac{2\pi}{N_t} + \lambda + \tfrac{\pi}{N_t}\bigr)$.

### Serrated chip force modulation for Ti-6Al-4V

Adiabatic shear bands in titanium produce segmented chips at a **segmentation frequency** $f_{\text{seg}} = V_{\text{chip}}/L_s$, where $L_s$ is the shear band spacing (typically **50–200 μm** for Ti-6Al-4V, decreasing with cutting speed). The force signal acquires a high-frequency modulation:

$$F(t) = F_{\text{mechanistic}}(t)\bigl[1 + \delta\sin(2\pi f_{\text{seg}}\,t)\bigr]$$

with amplitude modulation factor $\delta \approx 0.05\text{–}0.20$. The Calamaz-modified Johnson-Cook model (2008) captures the underlying strain softening that drives segmentation:

$$\sigma = \bigl[A + B\varepsilon^n / \exp(\varepsilon^a)\bigr]\bigl[1 + C\ln(\dot\varepsilon/\dot\varepsilon_0)\bigr]\bigl[1 - T^{*m}\bigr]\bigl[D + (1-D)\tanh(1/(\varepsilon+p)^r)\bigr]$$

with additional softening parameters $a \approx 2.0$, $D \approx 0.5\text{–}1.0$, $p \approx 0.05\text{–}0.2$, $r \approx 2.0\text{–}7.0$.

---

## 2. Constitutive material models and shear-angle prediction

### The Johnson-Cook constitutive equation

The standard JC model relates flow stress to strain, strain rate, and temperature:

$$\boxed{\sigma = (A + B\,\varepsilon_p^{\,n})\Bigl(1 + C\ln\frac{\dot\varepsilon_p}{\dot\varepsilon_0}\Bigr)\bigl(1 - T^{*m}\bigr)}$$

where $T^* = (T - T_{\text{room}})/(T_{\text{melt}} - T_{\text{room}})$ is the homologous temperature, $\dot\varepsilon_0$ is the reference strain rate (typically 1.0 s⁻¹), and $A$, $B$, $n$, $C$, $m$ are calibrated parameters.

**Calibrated JC parameters:**

| Alloy | Source | A (MPa) | B (MPa) | n | C | m | T_melt (°C) |
|-------|--------|---------|---------|------|-------|------|-------------|
| **Ti-6Al-4V** | Lee & Lin 1998 | 724.7 | 683.1 | 0.47 | 0.035 | 1.0 | 1660 |
| Ti-6Al-4V | Meyer & Kleponis 2001 | 862.5 | 331.2 | 0.34 | 0.012 | 0.8 | 1660 |
| Ti-6Al-4V | Calamaz et al. 2008 | 968.0 | 380.0 | 0.421 | 0.0197 | 0.577 | 1660 |
| **Inconel 718** | Iturbe et al. 2017 | 1241 | 622 | 0.6522 | 0.0134 | 1.3 | 1300 |
| Inconel 718 | DeMange et al. 2009 | 1290 | 895 | 0.526 | 0.016 | 1.55 | 1297 |
| **Waspaloy** | Approx. from Ni-base | ~900 | ~800 | 0.55 | 0.015 | 1.2 | 1330 |
| **AISI 4340** | Johnson & Cook 1985 | 792 | 510 | 0.26 | 0.014 | 1.03 | 1520 |

The Lee & Lin and Meyer & Kleponis parameter sets are most widely used for FEM simulation of Ti-6Al-4V machining. Published Waspaloy-specific JC parameters are scarce; Inconel 718 parameters with adjusted yield stress ($A \approx 900\text{–}1000$ MPa) provide a reasonable starting point.

### Zerilli-Armstrong model for HCP titanium

The Z-A model for HCP metals captures the dislocation-mechanics-based thermal activation:

$$\sigma = C_0 + C_1\exp(-C_3 T + C_4 T\ln\dot\varepsilon) + C_5\,\varepsilon^n$$

Calibrated parameters for Ti-6Al-4V (Zerilli & Armstrong 1996): $C_0 = 1060$ MPa, $C_1 = 960$ MPa, $C_3 = 6.0 \times 10^{-4}$ K⁻¹, $C_4 = 1.3 \times 10^{-4}$ K⁻¹, $C_5 = 850$ MPa, $n = 0.5$. The modified Z-A form (Meyer 2006, ARL-TR-3578) replaces the $C_5\varepsilon^n$ hardening term with a recovery-saturation form $C_2\sqrt{\varepsilon}\,[1-\exp(-\varepsilon/\varepsilon_r)]^{1/2}$, achieving **~1% average error** in predicted stress at strain rates 1000–50,000 s⁻¹.

### Oxley's predictive machining theory

Oxley's model predicts shear angle $\phi$, shear-plane temperature $T_{AB}$, and forces from first principles without empirical cutting-force coefficients. The key equations are:

**Shear-plane strain and strain rate:**
$$\varepsilon_{AB} = \frac{\cos\alpha}{\sqrt{3}\,\sin\phi\,\cos(\phi - \alpha)}, \qquad \dot\varepsilon_{AB} = \frac{C_0\,V_s}{\sqrt{3}\,\Delta s_1}$$

where $V_s = V\cos\alpha/\cos(\phi-\alpha)$ is the shear velocity, $\Delta s_1 = t_1 C_0/(2\sin\phi)$ is the shear zone half-thickness, and $C_0 \approx 5.9$.

**Shear-plane temperature:**
$$T_{AB} = T_{\text{room}} + \frac{(1-\Gamma)\,F_s\,V_s}{\rho\,c_p\,V\,t_1\,w}$$

where $\Gamma$ is the heat partition fraction to the workpiece, a function of the non-dimensional thermal number $R_T = \rho c_p V t_1 / k$:
- If $R_T\tan\phi < 0.04$: $\Gamma = 0.5 - 0.35\log_{10}(R_T\tan\phi)$
- If $0.04 \le R_T\tan\phi \le 10$: $\Gamma = 0.3 - 0.15\log_{10}(R_T\tan\phi)$

**Force equations:**
$$F_s = \frac{k_{AB}\,t_1\,w}{\sin\phi}, \quad F_c = \frac{F_s\cos(\beta_f - \alpha)}{\cos(\phi + \beta_f - \alpha)}, \quad F_t = \frac{F_s\sin(\beta_f - \alpha)}{\cos(\phi + \beta_f - \alpha)}$$

where $k_{AB} = \sigma_{AB}/\sqrt{3}$ is the shear flow stress evaluated from the JC model at $(\ \varepsilon_{AB},\,\dot\varepsilon_{AB},\,T_{AB})$.

**Iterative solution procedure:** For each trial shear angle $\phi$ (5°–45°) and trial $C_0$ (2–10), compute strains, temperatures, and stresses, then check two equilibrium conditions simultaneously: (1) normal stress at the shear-plane boundary matches $k_{AB}[1 + \pi/2 - 2\alpha - 2C_0 n_{\text{eq}}]$, and (2) interface shear stress equals material shear strength at the rake-face temperature. The equivalent strain hardening index is $n_{\text{eq}} = Bn\varepsilon^{n-1}/(A + B\varepsilon^n)$.

### Adiabatic shear band instability criterion

The **Recht criterion** (1964) for thermoplastic instability — the onset of serrated chip formation — states:

$$\boxed{\frac{\partial\sigma}{\partial\varepsilon}\bigg|_T + \frac{\partial\sigma}{\partial T}\bigg|_\varepsilon \cdot \frac{\beta_{TQ}\,\sigma}{\rho\,c_p} \le 0}$$

where $\beta_{TQ} \approx 0.9$ is the Taylor-Quinney coefficient. Using JC partial derivatives:
$$\frac{\partial\sigma}{\partial\varepsilon} = Bn\varepsilon^{n-1}(1+C\ln\dot\varepsilon/\dot\varepsilon_0)(1-T^{*m}), \quad \frac{\partial\sigma}{\partial T} = -(A+B\varepsilon^n)(1+C\ln\dot\varepsilon/\dot\varepsilon_0)\frac{mT^{*m-1}}{T_{\text{melt}}-T_{\text{room}}}$$

Instability occurs when thermal softening overcomes strain hardening. Semiatin and Rao (1983) derived a critical cutting speed for serrated chip onset:

$$V_{\text{cr}} = \frac{k\,\rho\,c_p\,\Delta\gamma^2\sin\phi}{\beta_{TQ}\,\tau\cos\alpha\,t_1\,|\partial\tau/\partial T|}$$

For Ti-6Al-4V, the low thermal conductivity ($k = 6.7$ W/m·K) produces extremely low $V_{\text{cr}}$, meaning **serrated chips form at virtually all practical cutting speeds**.

---

## 3. Tool wear ordinary differential equations

### Usui's wear rate ODE

The foundational wear model treats material removal from the tool as a thermally activated adhesive process:

$$\boxed{\frac{dW}{dt} = A\,\sigma_n\,V_s\,\exp\!\Bigl(-\frac{B}{T_{\text{int}}}\Bigr)}$$

where $W$ is local wear depth (mm), $\sigma_n$ is normal contact stress (MPa), $V_s$ is sliding velocity (m/s), $T_{\text{int}}$ is absolute interface temperature (K), and $A$, $B$ are calibration constants. For flank wear, the wear land width evolves as:

$$VB(t) = \int_0^t A\,\sigma_n(t')\,V_s(t')\,\exp\!\Bigl(-\frac{B}{T_{\text{int}}(t')}\Bigr)\,dt'$$

**Calibrated Usui constants:**

| Tool/Workpiece | Temperature Range | A | B (K) |
|----------------|-------------------|---|-------|
| WC on general metals | T < 1150 K | **7.8 × 10⁻⁹** | **5302** |
| WC on general metals | T ≥ 1150 K | **1.198 × 10⁻²** | **2.195 × 10⁴** |
| WC on Inconel 718 | All T | **1.08 × 10⁻¹²** | **8900** |

For Ti-6Al-4V, a combined multi-mechanism form captures adhesion, abrasion, and diffusion simultaneously:

$$\frac{dW}{dt} = A_{\text{adh}}\,\sigma_n\,V_s\,\exp\!\Bigl(-\frac{B_{\text{adh}}}{T}\Bigr) + A_{\text{abr}}\,\sigma_n\,V_s + A_{\text{diff}}\,\exp\!\Bigl(-\frac{B_{\text{diff}}}{T}\Bigr)$$

### Takeyama-Murata wear model

This model separates mechanical and diffusion wear:

$$\frac{dVB}{dt} = G(V, f) + D_0\exp\!\Bigl(-\frac{E_a}{RT}\Bigr)$$

where the first term $G(V,f) = \alpha V^\beta f^\gamma$ represents temperature-independent abrasive wear proportional to sliding distance, and the second term is an **Arrhenius diffusion term** that dominates above ~800°C for WC tools. For WC-Co tools cutting titanium, Co binder outward-diffusion into the chip is the primary diffusion mechanism, with activation energy $Q \approx 150\text{–}200$ kJ/mol.

### Taylor's tool life and its differential connection

The classical Taylor equation $VT^n = C$ (with typical $n = 0.15\text{–}0.25$ for uncoated carbide on Ti-6Al-4V) is the **integrated consequence** of the steady-state wear rate ODE. The extended form $VT^n f^a d^b = C_{\text{ext}}$ captures feed and depth dependence. Pálmai unified all three wear stages (break-in, steady, accelerating) into a single autonomous ODE:

$$\frac{dVB}{dt} = \bigl[\alpha_1\,VB^{m_1} + \alpha_2\exp(\alpha_3\,VB)\bigr]\,V^p$$

When integrated to $VB = VB_{\text{crit}}$, this recovers the Taylor relationship.

### Wear-force feedback coupling

Flank wear creates additional ploughing/rubbing forces that close the feedback loop. The edge force coefficients grow linearly with wear:

$$K_{te}(VB) = K_{te,0} + C_{te}\,VB, \qquad K_{re}(VB) = K_{re,0} + C_{re}\,VB$$

and the shearing coefficients follow a quadratic trend:

$$K_{tc}(VB) = a_{tc} + b_{tc}\,VB + c_{tc}\,VB^2$$

Experimental data show force coefficients increasing by **57–495%** over the tool life. The simplified Waldorf slip-line model gives the additional tangential and feed forces as $\Delta F_t = \sigma_w\,VB\,a_p\,[1 + \mu\tan\theta_w]$ and $\Delta F_f = \sigma_w\,VB\,a_p\,[\mu + \tan\theta_w]$, where $\sigma_w$ is the workpiece material flow stress on the wear land.

This creates a **positive feedback loop**: wear → increased forces → increased temperature → accelerated wear rate. The state-space formulation treats VB as a slow state variable evolving over minutes, coupled to the fast kHz-rate cutting dynamics through the force coefficients.

---

## 4. Thermal differential equations and heat partition

### Heat generation and partition

Total cutting power is $P = F_c V_c$, partitioned among chip, tool, and workpiece. The Loewen-Shaw heat partition fraction $R_f$ going into the chip from the rake-face friction source is:

$$R_f = \frac{1}{1 + \dfrac{0.946\sqrt{\pi}\,k_w\sqrt{Pe}}{2\,k_t}}$$

where $Pe = Va/(4\alpha)$ is the Peclet number, $a$ is the contact half-length, $k_w$ and $k_t$ are workpiece and tool thermal conductivities, and $\alpha$ is workpiece thermal diffusivity. For Ti-6Al-4V with $k_w = 6.7$ W/m·K (versus ~50 W/m·K for steel), the **thermal diffusivity is only 2.87 × 10⁻⁶ m²/s** — about 4.6× lower than steel — concentrating heat at the tool-chip interface and driving tool-face temperatures to **800–1100°C** even at moderate speeds.

### Komanduri-Hou moving heat source formulation

The temperature field from the primary shear zone (an inclined band source at shear angle $\phi$, moving at cutting velocity $V$) uses Hahn's oblique moving-source kernel. For an infinitesimal segment $dl_i$ of the shear plane, the temperature rise at point $M(x,y)$ is:

$$dT_M = \frac{q_s\,dl_i}{2\pi k}\exp\!\Bigl[-\frac{V(x-x_i)}{2\alpha}\Bigr]\,K_0\!\Bigl[\frac{V R_i}{2\alpha}\Bigr]$$

where $K_0$ is the **modified Bessel function of the second kind (zeroth order)**, $R_i = \sqrt{(x-x_i)^2 + (y-y_i)^2}$, and along the shear plane $x_i = l_i\cos\phi$, $y_i = l_i\sin\phi$. The total shear-zone temperature is obtained by integration over the shear plane length $L_s$:

$$T(x,y) = \frac{q_s}{2\pi k}\int_0^{L_s}\exp\!\Bigl[-\frac{V(x - l_i\cos\phi)}{2\alpha}\Bigr]\,K_0\!\Bigl[\frac{V\sqrt{(x - l_i\cos\phi)^2 + (y - l_i\sin\phi)^2}}{2\alpha}\Bigr]\,dl_i$$

Image heat sources enforce the adiabatic boundary condition at the workpiece free surface. The secondary-zone (rake-face friction) temperature uses the same kernel but with chip velocity $V_{\text{chip}}$ and a non-uniform heat flux $q_f(x_i)$ along the contact length $l_c$. The total temperature is the superposition $T_{\text{total}} = T_{\text{ambient}} + \Delta T_{\text{shear}} + \Delta T_{\text{friction}}$.

### Energy equation in the deformation zones

The full energy equation governing temperature in the primary shear zone is:

$$\rho c_p\Bigl(\frac{\partial T}{\partial t} + V_x\frac{\partial T}{\partial x} + V_y\frac{\partial T}{\partial y}\Bigr) = k\Bigl(\frac{\partial^2 T}{\partial x^2} + \frac{\partial^2 T}{\partial y^2}\Bigr) + \beta_{TQ}\,\bar\sigma\,\dot{\bar\varepsilon}}$$

where $\beta_{TQ} \approx 0.9$ converts plastic work to heat. In the secondary zone, the frictional heat source intensity is $q_{\text{friction}} = \tau_{\text{int}} V_{\text{chip}}$, where $\tau_{\text{int}}$ equals the interfacial shear stress (friction coefficient × normal stress in the sliding zone, or material shear yield strength in the sticking zone).

### Tool temperature ODE for interrupted milling

In milling, each tooth engages intermittently, creating cyclic thermal loading. The lumped thermal model gives:

**Heating phase** (tooth in cut, $0 \le t \le t_{\text{cut}}$):
$$\rho_t c_t V_t\frac{dT}{dt} = q_{\text{in}}\,A_c - h_{\text{eff}}\,A_s\,(T - T_{\text{amb}})$$

**Cooling phase** (tooth out of cut, $t_{\text{cut}} \le t \le t_{\text{cut}} + t_{\text{cool}}$):
$$\rho_t c_t V_t\frac{dT}{dt} = -h_{\text{cool}}\,A_s\,(T - T_{\text{amb}})$$

The heating phase solution is $T(t) = T_{\text{amb}} + q_{\text{in}}/h_{\text{eff}}\,[1 - e^{-t/\tau_h}] + (T_0 - T_{\text{amb}})e^{-t/\tau_h}$ with thermal time constant $\tau_h = \rho_t c_t L_c / h_{\text{eff}}$. At **cyclic steady state**, the peak and trough temperatures converge to:

$$T_{\text{max,ss}} = T_{\text{amb}} + \frac{(q_{\text{in}}/h_{\text{eff}})(1 - e^{-t_{\text{cut}}/\tau_h})}{1 - e^{-t_{\text{cut}}/\tau_h}\,e^{-t_{\text{cool}}/\tau_c}}$$

$$T_{\text{min,ss}} = T_{\text{amb}} + (T_{\text{max,ss}} - T_{\text{amb}})\,e^{-t_{\text{cool}}/\tau_c}$$

where $t_{\text{cut}} = \theta_{\text{eng}}/(2\pi n_s)$ and $t_{\text{cool}} = (2\pi - \theta_{\text{eng}})/(2\pi n_s)$ depend on the engagement angle and spindle speed $n_s$. This thermal cycling drives **thermal fatigue** in milling tools — a phenomenon absent in continuous turning.

**Thermal properties comparison (room temperature):**

| Property | Ti-6Al-4V | Inconel 718 | AISI 1045 Steel |
|----------|-----------|-------------|-----------------|
| k (W/m·K) | **6.7** | 11.4 | ~50 |
| ρ (kg/m³) | 4430 | 8190 | 7860 |
| c_p (J/kg·K) | 526 | 435 | 486 |
| α (×10⁻⁶ m²/s) | **2.87** | 3.20 | 13.1 |

---

## 5. Regenerative chatter stability via delay differential equations

### The governing DDE

Regenerative chatter arises because each tooth cuts a surface left by the previous tooth. The dynamic chip thickness includes a regenerative term:

$$h_j(t) = \bigl[\Delta x(t)\sin\phi_j(t) + \Delta y(t)\cos\phi_j(t)\bigr]\,g(\phi_j)$$

where $\Delta x(t) = x(t) - x(t-\tau)$ and $\Delta y(t) = y(t) - y(t-\tau)$, with **time delay** $\tau = 60/(N_t \cdot n)$ equal to the tooth-passing period. The 2-DOF equations of motion are:

$$\ddot{q}_x + 2\zeta_x\omega_{nx}\dot{q}_x + \omega_{nx}^2 q_x = \frac{\omega_{nx}^2}{k_x}\,F_x(t)$$

$$\ddot{q}_y + 2\zeta_y\omega_{ny}\dot{q}_y + \omega_{ny}^2 q_y = \frac{\omega_{ny}^2}{k_y}\,F_y(t)$$

where the cutting forces depend on the **time-periodic directional dynamic force coefficient matrix** $[A(t)]$:

$$\begin{Bmatrix}F_x \\ F_y\end{Bmatrix} = \frac{1}{2}a_p K_t [A(t)]\begin{Bmatrix}\Delta x \\ \Delta y\end{Bmatrix}$$

The matrix elements, summed over all engaged teeth, are:

$$a_{xx} = \sum_j g_j[-\sin 2\phi_j + K_r(1-\cos 2\phi_j)], \qquad a_{xy} = \sum_j g_j[-(1+\cos 2\phi_j) + K_r\sin 2\phi_j]$$

$$a_{yx} = \sum_j g_j[(1-\cos 2\phi_j) - K_r\sin 2\phi_j], \qquad a_{yy} = \sum_j g_j[\sin 2\phi_j - K_r(1+\cos 2\phi_j)]$$

where $K_r = K_{rc}/K_{tc}$ is the force ratio. This is a **linear time-periodic delay differential equation** — the defining mathematical object for milling chatter.

### Altintas-Budak zero-order approximation for stability lobes

Taking only the zeroth Fourier harmonic $[A_0] = (N_t/2\pi)[\alpha_{ij}]$ with:

$$\alpha_{xx} = \tfrac{1}{2}[\cos 2\phi - 2K_r\phi + K_r\sin 2\phi]\Big|_{\phi_{\text{st}}}^{\phi_{\text{ex}}}, \quad \alpha_{xy} = \tfrac{1}{2}[-\sin 2\phi - 2\phi + K_r\cos 2\phi]\Big|_{\phi_{\text{st}}}^{\phi_{\text{ex}}}$$

$$\alpha_{yx} = \tfrac{1}{2}[-\sin 2\phi + 2\phi + K_r\cos 2\phi]\Big|_{\phi_{\text{st}}}^{\phi_{\text{ex}}}, \quad \alpha_{yy} = \tfrac{1}{2}[-\cos 2\phi - 2K_r\phi - K_r\sin 2\phi]\Big|_{\phi_{\text{st}}}^{\phi_{\text{ex}}}$$

the stability boundary is found from the eigenvalue problem $\det[I + \Lambda\,[A_0]\,[\Phi(i\omega_c)]] = 0$, where $\Lambda = -(N_t/4\pi)\,a_p\,K_t\,(1 - e^{-i\omega_c\tau})$ and $\Phi(i\omega)$ is the FRF matrix. The critical axial depth of cut and corresponding spindle speeds are:

$$\boxed{a_{p,\text{lim}} = -\frac{2\pi\,\Lambda_R}{N_t\,K_t}(1 + \kappa^2)}, \qquad n = \frac{60\,\omega_c}{N_t\,(2k_l\pi + \varepsilon)}$$

where $\kappa = \Lambda_I/\Lambda_R$ and $\varepsilon = \pi - 2\arctan\kappa$. The integer $k_l = 0, 1, 2, \ldots$ generates successive **stability lobes**. The algorithm scans chatter frequency $\omega_c$ through the negative-real-part region of the FRF, computes $\Lambda$ from the eigenvalue problem, and plots $(n, a_{p,\text{lim}})$.

### Semi-discretization method (Insperger-Stépán)

For higher accuracy (capturing period-doubling bifurcations missed by ZOA), the delay period $\tau$ is divided into $m$ intervals of $\Delta t = \tau/m$. The DDE is reformulated in state-space as $\dot{q}(t) = [L_i]\,q(t) + [R_i]\,q(t-\tau)$, with the delayed term approximated by averaging: $q(t-\tau) \approx (q_{i-m+1} + q_{i-m})/2$. The discrete map becomes:

$$q_{i+1} = e^{[L_i]\Delta t}\,q_i + \tfrac{1}{2}(e^{[L_i]\Delta t} - I)[L_i]^{-1}[R_i](q_{i-m+1} + q_{i-m})$$

An extended state vector $z_i = [q_i, q_{i-1}, \ldots, q_{i-m}]^T$ yields the transition matrix $z_{i+1} = [B_i]z_i$. The **monodromy matrix** $\Phi = B_m B_{m-1}\cdots B_1$ governs stability via Floquet theory: the system is stable if and only if **all eigenvalues** (Floquet multipliers) $\mu$ satisfy $|\mu| < 1$. Eigenvalues crossing the unit circle as complex conjugates indicate Hopf bifurcation (classical chatter); $\mu = -1$ indicates period-doubling instability.

### Process damping at low speeds in titanium

At the low cutting speeds typical for Ti-6Al-4V (50–90 m/min), flank-face contact with the undulating machined surface creates an additional velocity-dependent damping force:

$$F_{\text{pd}} = a_p\,C_p\,\frac{\dot{r}}{v_c}$$

where $C_p$ is the **process damping coefficient** (typical values for Ti-6Al-4V: $C_p \approx 1\text{–}5 \times 10^6$ N/m). This effectively increases the damping ratio: $\zeta_{\text{eff}} = \zeta + a_p C_p/(2v_c m\omega_n)$. At low $v_c$, this term dominates and substantially raises the stable depth of cut above the ZOA prediction. As tool wear progresses ($VB$ increases), the flank contact area grows, further increasing $C_p$ and shifting stability boundaries upward at low speeds while potentially lowering them at high speeds through changed cutting-force coefficients.

---

## 6. Surface roughness generation equations

### Kinematic roughness from tool geometry

The theoretical peak-to-valley roughness from the circular arc of the tool nose radius $r_\varepsilon$ intersecting at feed spacing $f_t$ is:

$$R_t = r_\varepsilon - \sqrt{r_\varepsilon^2 - f_t^2/4} \approx \frac{f_t^2}{8\,r_\varepsilon} \quad (f_t \ll r_\varepsilon)$$

The arithmetic average roughness is $R_a \approx f_t^2/(32\,r_\varepsilon)$.

### Minimum chip thickness and the Brammertz correction

When the instantaneous chip thickness falls below the minimum $h_{\min} \approx k_h\,r_e$ (where $r_e$ is the cutting edge radius and $k_h \approx 0.2\text{–}0.35$ for Ti-6Al-4V), no chip forms and material undergoes elastic-plastic deformation. The Brammertz model corrects the kinematic roughness:

$$R_{zt} = \frac{f_t^2}{8\,r_\varepsilon} + \frac{h_{\min}}{2}\Bigl(1 + \frac{h_{\min}\,r_\varepsilon}{f_t^2}\Bigr)$$

Elastic recovery (springback) adds a residual height $h_{\text{elastic}} \approx r_e(1 - H/E)$, and lateral material side flow creates ridges proportional to $C_{\text{sf}}\,h_{\min}\,(r_e/f_t)$.

### Vibration-superimposed surface profile

The actual machined surface is the **minimum envelope** of all tooth trajectories including vibration:

$$z_{\text{surface}}(x) = \min_j\{y_j(t) \mid x_j(t) = x\}$$

where $x_j(t) = v_f t + R\sin(\Omega t + j\cdot 2\pi/N_t) + \delta x_j(t)$ and $\delta x_j$ is the vibration displacement. The total roughness combines kinematic, vibration, elastic, and ploughing contributions:

$$R_{a,\text{total}} = \frac{f_t^2}{32\,r_\varepsilon} + C_{\text{vib}}\,A_{\text{vib}} + \Delta R_{a,\text{elastic}} + \Delta R_{a,\text{sideflow}}$$

### Tool wear degradation

Flank wear increases roughness through additional ploughing and larger vibration amplitudes:

$$R_a(VB) = \frac{f_t^2}{32\,r_\varepsilon} + K_w\,VB^{n_w}$$

where $K_w$ is a wear-roughness coefficient and $n_w \approx 0.5\text{–}1.0$. Empirically, $R_a$ roughly doubles as $VB$ progresses from fresh to 0.2 mm for Ti-6Al-4V. At severe wear levels, surface integrity degrades: residual stresses shift from compressive (beneficial) to tensile, and hard recrystallized white layers (1–5 μm thick) may form from severe thermo-mechanical loading.

---

## 7. The complete coupled state-space system

### State vector and coupled ODEs

The full digital twin state vector is:

$$\mathbf{x} = [q_x,\; q_y,\; \dot{q}_x,\; \dot{q}_y,\; T_{\text{tool}},\; T_{\text{wp}},\; VB,\; W_{\text{crater}}]^T$$

with input vector $\mathbf{u} = [\Omega, f_t, a_p, a_e]^T$. The coupled system $\dot{\mathbf{x}} = \mathbf{f}(\mathbf{x}, \mathbf{x}_\tau, \mathbf{u}, t)$ is:

**Fast structural dynamics** (~kHz):
$$\ddot{q}_x = \frac{1}{m_x}\bigl[-c_x\dot{q}_x - k_x q_x + F_x\bigl(h(\mathbf{q}, \mathbf{q}_\tau),\, K_c(VB, T_{\text{tool}}),\, a_p\bigr)\bigr]$$
$$\ddot{q}_y = \frac{1}{m_y}\bigl[-c_y\dot{q}_y - k_y q_y + F_y\bigl(h(\mathbf{q}, \mathbf{q}_\tau),\, K_c(VB, T_{\text{tool}}),\, a_p\bigr)\bigr]$$

**Medium thermal dynamics** (~Hz):
$$\frac{dT_{\text{tool}}}{dt} = \frac{1}{\rho_t c_t V_t}\bigl[\eta_{\text{tool}}\,P_{\text{cutting}} - h_{\text{conv}}\,A_s\,(T_{\text{tool}} - T_{\text{amb}}) - h_{\text{cond}}\,A_c\,(T_{\text{tool}} - T_{\text{wp}})\bigr]$$

**Slow wear dynamics** (~mHz):
$$\frac{dVB}{dt} = A\,\sigma_n(F, VB)\,V_s\,\exp\!\Bigl(-\frac{B}{T_{\text{int}}(T_{\text{tool}}, F)}\Bigr)$$

The **coupling pathways** form a closed loop: (1) forces depend on wear $VB$ and temperature $T$ through $K_{tc}(VB, T) = K_{tc,0}[1 + C_w VB/VB_0][1 + C_T(T - T_{\text{ref}})/T_{\text{ref}}]$; (2) temperature depends on cutting power $P = F_c v_c$; (3) wear rate depends on both contact stress (from forces) and temperature (from thermal state). Structural vibrations modulate the chip thickness at kHz rates, creating force oscillations that appear as high-frequency temperature fluctuations, while wear evolves quasi-statically over minutes.

### Multi-timescale integration strategy

The system exhibits a **stiffness ratio exceeding 10⁶** between the fastest (vibration at ~5 kHz) and slowest (wear over ~30 min) dynamics. The singular perturbation formulation is:

$$\varepsilon_1\,\dot{\mathbf{x}}_{\text{fast}} = \mathbf{f}_1(\mathbf{x}_{\text{fast}}, \mathbf{x}_{\text{med}}, \mathbf{x}_{\text{slow}}, \mathbf{u}, t)$$
$$\varepsilon_2\,\dot{\mathbf{x}}_{\text{med}} = \mathbf{f}_2(\bar{\mathbf{x}}_{\text{fast}}, \mathbf{x}_{\text{med}}, \mathbf{x}_{\text{slow}}, \mathbf{u})$$
$$\dot{\mathbf{x}}_{\text{slow}} = \mathbf{f}_3(\bar{\mathbf{x}}_{\text{fast}}, \mathbf{x}_{\text{med}}, \mathbf{x}_{\text{slow}}, \mathbf{u})$$

where $\varepsilon_1 \sim 10^{-4}$ s, $\varepsilon_2 \sim 1$ s, and $\bar{\mathbf{x}}_{\text{fast}}$ denotes per-revolution averaged fast states. The practical **multi-rate integration scheme** operates as follows:

- **Fast loop** (explicit RK4 or semi-discretization, $\Delta t_1 \sim 10^{-5}$ s): Resolve structural vibration, regenerative chatter DDE, and instantaneous forces for one tooth-passing period.
- **Medium loop** (implicit Euler or BDF, $\Delta t_2 \sim 10^{-2}$ s): Update tool temperature using average cutting power from the fast loop.
- **Slow loop** (explicit Euler, $\Delta t_3 \sim 1\text{–}60$ s): Update wear state using current temperature and average contact stresses; feed updated $VB$ and $T$ back to force coefficients.

For real-time execution, IMEX (implicit-explicit) methods handle the stiffness: explicit for the fast structural dynamics, implicit for the coupled thermal-wear subsystem. The delay term $\mathbf{q}(t-\tau)$ requires a circular buffer storing past states. Reduced-order modeling via Proper Orthogonal Decomposition (thermal fields) and Craig-Bampton reduction (structural dynamics) brings computational cost within real-time bounds.

### The output equation

The observable quantities relate to states via:

$$y_1 = F_x = \sum_j[-F_{t,j}\cos\phi_j - F_{r,j}\sin\phi_j], \qquad y_2 = F_y, \qquad y_3 = F_z$$
$$y_4 = T_{\text{measured}} = T_{\text{tool}} + v_T, \qquad y_5 = \ddot{q}_x \text{ (accelerometer)}$$
$$y_6 = P_{\text{spindle}} = (F_t R + \tau_{\text{friction}})\Omega$$
$$y_7 = R_a \approx \frac{f_t^2}{32 r_\varepsilon}\bigl[1 + k_{Ra}(VB/VB_{\text{ref}})^{n_{Ra}} + k_{\text{vib}}(q_{\text{amp}}/q_{\text{ref}})^2\bigr]$$

---

## 8. Cybersecurity: physics-based command verification and anomaly detection

### Digital twin as intrusion detection system

The digital twin generates a **force prediction envelope** for each G-code block: for commanded parameters $(X, Y, Z, F, S)$, the mechanistic model computes expected $F_{\text{pred}}(t) \pm \delta F(t)$, where $\delta F = k_\sigma\,\sigma_F$ (typically $k_\sigma = 3$ for 99.7% confidence) accounts for model uncertainty, material variation, and measurement noise. The verification check $|F_{\text{measured}} - F_{\text{predicted}}| < \delta F_{\text{threshold}}$ runs continuously. Because the physics model constrains the relationship $F = K_c a_p h \cdot g(\phi)$, $P = F_c v_c$, $a = F/m$ simultaneously, an attacker would need to spoof all correlated sensor channels consistently with the physics — an extremely high barrier.

### Residual-based statistical detection

The residual signal $\mathbf{r}(t) = \mathbf{y}_{\text{measured}}(t) - \hat{\mathbf{y}}_{\text{predicted}}(t)$ is monitored by three complementary tests:

**CUSUM** (sensitive to persistent small shifts):
$$S_k^+ = \max(0,\; S_{k-1}^+ + r_k/\sigma - k_{\text{slack}}), \qquad \text{alarm if } S_k^+ > h$$

with allowance $k_{\text{slack}} \approx 0.5\text{–}1.0$ and threshold $h$ set by desired average run length $ARL_0$.

**EWMA** (smooth tracking of mean shifts):
$$z_k = \lambda\,r_k + (1-\lambda)\,z_{k-1}, \qquad \text{alarm if } |z_k - \mu_0| > L\sigma\sqrt{\lambda/(2-\lambda)}$$

with smoothing parameter $\lambda \in [0.05, 0.25]$ and width $L \approx 2.5\text{–}3.0$.

**Multivariate chi-squared** (for the full residual vector):
$$\chi_k^2 = \mathbf{r}_k^T\,\Sigma_r^{-1}\,\mathbf{r}_k, \qquad \text{alarm if } \chi_k^2 > \chi_{\alpha,p}^2$$

where $\Sigma_r$ is the residual covariance estimated from normal-operation training data and $p$ is the number of residual channels.

### Attack signatures and detection mapping

Each attack vector produces a characteristic residual pattern: modified G-code coordinates cause systematic force deviations proportional to the coordinate error across all channels; altered feed rates produce force residuals scaling as $\Delta F/F \approx \Delta f/f$; spoofed force sensors create inconsistency with acceleration and power channels (cross-modal physics violation). The digital twin's physics-based baseline cannot be fooled by attacks on a single sensor because the multi-modal consistency check ($F$, $P$, $a$, $T$ must all satisfy the governing equations simultaneously) provides inherent redundancy.

### Extended Kalman Filter for state estimation and security

The nonlinear machining system is monitored by an EKF with state $\hat{\mathbf{x}} = [VB, T_{\text{tool}}, \mathbf{q}]^T$:

$$\hat{\mathbf{x}}_{k|k-1} = \mathbf{f}(\hat{\mathbf{x}}_{k-1|k-1}, \mathbf{u}_k), \qquad P_{k|k-1} = F_k P_{k-1|k-1} F_k^T + Q_k$$
$$K_k = P_{k|k-1}H_k^T(H_k P_{k|k-1}H_k^T + R_k)^{-1}$$
$$\hat{\mathbf{x}}_{k|k} = \hat{\mathbf{x}}_{k|k-1} + K_k(\mathbf{y}_k - \mathbf{h}(\hat{\mathbf{x}}_{k|k-1}))$$
$$P_{k|k} = (I - K_k H_k)P_{k|k-1}$$

where $F_k = \partial\mathbf{f}/\partial\mathbf{x}$ and $H_k = \partial\mathbf{h}/\partial\mathbf{x}$ are the Jacobians. Under normal operation, the innovation $\mathbf{r}_k = \mathbf{y}_k - \mathbf{h}(\hat{\mathbf{x}}_{k|k-1})$ is distributed as $\mathcal{N}(0, S_k)$ with $S_k = H_k P_{k|k-1}H_k^T + R_k$. The **normalized innovation** $\tilde{r}_k = r_k/\sqrt{S_k}$ serves as the anomaly detection statistic, fed to CUSUM/EWMA for real-time monitoring. A cyber attack or workpiece defect (void → sudden force drop; hard inclusion → force spike; AM layer variation → periodic force modulation with period $\lambda_{\text{layer}}$) causes $\tilde{r}_k$ to deviate from the standard normal distribution, triggering detection.

### What the physics-based IDS can and cannot detect

The digital twin detects modified toolpaths, changed cutting parameters, tool-offset tampering, and single-sensor spoofing (via cross-modal inconsistency). It **cannot** detect stealthy attacks that remain within model uncertainty bounds ($|\Delta F| < \delta F$), which requires the attacker to have an equally accurate process model — a very high barrier. Compared to signature-based IDS (which miss novel attacks) and purely statistical anomaly IDS (which suffer high false-alarm rates), the physics-based approach offers both novelty detection and low false alarms because the residual distribution is constrained by physical law rather than learned from data alone.

---

## Conclusion: a unified dynamical system across six orders of magnitude in time

The digital twin of CNC milling is, at its mathematical core, a **nonlinear, time-periodic, multi-scale delay differential-algebraic system**. The fastest dynamics — regenerative vibration and instantaneous cutting forces — oscillate at tooth-passing frequencies of 1–10 kHz and are governed by the 2-DOF DDE with the $[A(t)]$ directional coefficient matrix. The thermal state evolves 3–4 orders of magnitude more slowly, driven by the lumped energy ODE with cyclic heating and cooling every revolution. Tool wear, the slowest state, follows the Usui exponential ODE over minutes to hours, yet feeds back to every faster subsystem through force coefficients that grow monotonically with $VB$.

The key insight for implementation is that **all coupling is unidirectional within each timestep**: the slow states (wear, temperature) are quasi-frozen on the fast timescale, serving as parameters rather than variables. This permits the multi-rate integration scheme — fast explicit stepping for the DDE, slow implicit stepping for thermal-wear — that makes real-time execution feasible. The cybersecurity layer adds no additional dynamics; it operates purely on the innovation sequence of the Kalman filter, comparing the physics model's predictions against sensor reality. A discrepancy that persists beyond the CUSUM threshold indicates either a physical anomaly (defect, tool breakage) or a cyber-physical attack — both demanding immediate operator attention. The constitutive models (Johnson-Cook, Zerilli-Armstrong) supply the material behavior that anchors every force, temperature, and wear prediction to physical first principles rather than empirical curve fits, giving the digital twin its predictive authority over the full operating envelope of these difficult-to-machine alloys.
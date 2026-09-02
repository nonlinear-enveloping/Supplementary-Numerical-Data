# Supplementary Numerical Data for  
# "Nonlinear Enveloping for Membership-Function-Dependent Synthesis of T-S Fuzzy Systems"

## 1. Purpose

This repository provides the supplementary numerical data used to reproduce the polynomial representation and robust \(H_\infty\) controller synthesis for the unmanned surface vehicle (USV) example presented in the manuscript.

The data correspond to the theoretical procedure developed in Sections III-A–III-E and the USV example in Section IV-A of the manuscript. In particular, the repository provides:

- the sampled augmented upper and lower envelope functions;
- the complete fifth-order polynomial coefficients;
- the monomial-index convention associated with these coefficients;
- the continuous local residual bounds used in the LMIs; and
- summary information for checking the numerical reproduction.

The repository contains numerical data only. No implementation source code is included.

---

## 2. Files

The repository contains the following files:

- `Gamma_upper.csv`  
  Complete polynomial coefficients of the augmented upper-envelope functions \(\hat f_{u,ij}(z)\).

- `Gamma_lower.csv`  
  Complete polynomial coefficients of the augmented lower-envelope functions \(\hat f_{d,ij}(z)\).

- `monomial_index.csv`  
  Mapping between each coefficient column and the corresponding multi-index \(\nu=(\nu_1,\nu_2)\) in the polynomial representation.

- `fitted_coefficients.mat`  
  MATLAB-format file containing the same coefficient matrices and associated numerical metadata.

- `fitting_samples.csv`  
  The \(N_s=400\) sampled values of the augmented upper and lower envelope functions used for least-squares fitting.

- `residual_bounds.csv`  
  Certified lower and upper approximation-residual bounds over the \(N_\Omega=196\) continuous local sub-domains.

- `fitting_summary.csv`  
  Main fitting, residual-bound, and normalization-validation statistics.

The following sections describe how these data correspond to the formulation in the manuscript and how they can be used to reproduce the USV synthesis result.

---

## 3. Step 1: USV T-S Model and Membership Functions

The USV state vector is

\[
x=[u,v,r,x,y,\psi]^T,
\]

where \(u\), \(v\), and \(r\) are the surge velocity, sway velocity, and yaw rate, respectively, and \(x\), \(y\), and \(\psi\) denote the north position, east position, and yaw angle.

The two premise variables used for the T-S fuzzy model are

\[
z_1=u=x_1\in[0.8,2.8], \qquad
z_2=r=x_3\in[-0.2,0.2].
\]

The four normalized membership functions used in the USV example are

\[
h_1(z_1,z_2)
=(0.5z_1-0.4)(0.5+2.5z_2),
\]

\[
h_2(z_1,z_2)
=(0.5z_1-0.4)(0.5-2.5z_2),
\]

\[
h_3(z_1,z_2)
=(1.4-0.5z_1)(0.5+2.5z_2),
\]

\[
h_4(z_1,z_2)
=(1.4-0.5z_1)(0.5-2.5z_2).
\]

They satisfy

\[
h_i(z)\ge 0,\qquad
\sum_{i=1}^{4}h_i(z)=1
\]

over the identified premise-variable domain.

The corresponding premise-domain vertices are ordered as

\[
(2.8,0.2),\quad
(2.8,-0.2),\quad
(0.8,0.2),\quad
(0.8,-0.2),
\]

for Rules 1–4, respectively.

The identified hydrodynamic parameters used to construct the nominal T-S model are

\[
m_{11}=82.8813,\qquad
m_{22}=83.4000,\qquad
m_{33}=9.8470,
\]

\[
d_{11}=-25.2160,\qquad
d_{11q}=33.9944,\qquad
d_{22}=66.1440,\qquad
d_{33}=10.1492.
\]

The matrices \(A_i\), \(B_i\), \(E_i\), \(C_i\), and the uncertainty matrices are constructed according to Section IV-A of the manuscript. A \(5\%\) perturbation of the identified hydrodynamic parameters is considered in the robust \(H_\infty\) synthesis.

---

## 4. Step 2: Radial Envelope Construction

### 4.1 Original MF space

Following Section III-A, the anchor point used in the USV example is

\[
h_0=h(0.8,0.2)
=
[0,0,1,0]^T.
\]

For an arbitrary \(z=(z_1,z_2)\), define

\[
\Lambda(z)=\|h(z)-h_0\|_2
\]

and, for \(\Lambda(z)>0\),

\[
e(z)=\frac{h(z)-h_0}{\Lambda(z)}.
\]

The upper and lower radial envelopes are defined by

\[
h_u(z)=h(z)+d_e(z)e(z),
\]

\[
h_d(z)=h(z)-d_e(z)e(z),
\]

corresponding to Eqs. (8)–(9) of the manuscript.

For the USV example,

\[
\delta=0.05
\]

and

\[
d_e(z)
=
\delta\Phi(\vartheta(z))d_{\max}(z),
\]

where \(d_{\max}(z)\) is defined by Eq. (21). The shape function in Eq. (23) becomes

\[
\Phi(z_1,z_2)
=
\sin\left(
\frac{z_1-0.8}{2.0}\pi
\right)
\sin\left(
\frac{z_2+0.2}{0.4}\pi
\right).
\]

This construction preserves the unit-sum and nonnegativity constraints of the probability simplex.

### 4.2 Augmented MF space

After applying the PDC controller, the product weighting functions are defined according to Eq. (30) as

\[
f_{ij}(z)=h_i(z)h_j(z),
\qquad i,j=1,\ldots,4.
\]

Hence,

\[
N_f=l^2=16.
\]

The augmented components are stored in row-major order,

\[
f_{11},f_{12},f_{13},f_{14},
f_{21},\ldots,f_{44}.
\]

Equivalently, the row index \(r\) of component \(f_{ij}\) is

\[
r=4(i-1)+j.
\]

The same radial construction described above is applied directly to the 16-dimensional augmented weighting vector \(f(z)\), as described in Section III-B, yielding

\[
f_{u,ij}(z),\qquad
f_{d,ij}(z).
\]

The file `fitting_samples.csv` contains the values of these 16 upper and 16 lower augmented envelope functions at the \(20\times20\) uniformly distributed fitting grid,

\[
N_s=400.
\]

These data can be used to independently verify the polynomial fitting described below.

---

## 5. Step 3: Polynomial Representation and Coefficient Data

Following Eq. (36), each augmented endpoint function is represented by a complete multivariate polynomial

\[
\hat f_{\sigma,ij}(z)
=
\sum_{\nu\in\mathcal L_{\eta}}
\Gamma_{\sigma,ij,\nu}
z_1^{\nu_1}z_2^{\nu_2},
\qquad
\sigma\in\{u,d\},
\]

where

\[
\mathcal L_{\eta}
=
\left\{
(\nu_1,\nu_2):
\nu_1+\nu_2\le\eta
\right\}.
\]

For the reported USV result,

\[
\eta=5.
\]

Therefore, each complete fifth-order bivariate polynomial contains

\[
\frac{(\eta+1)(\eta+2)}{2}
=
21
\]

monomial coefficients.

The exponent pair \((\nu_1,\nu_2)\) associated with each coefficient column is provided in

`monomial_index.csv`.

The coefficient matrices are provided in

`Gamma_upper.csv`

and

`Gamma_lower.csv`.

For example, if row \(r\) corresponds to \(f_{ij}\), its upper polynomial is reconstructed as

\[
\hat f_{u,ij}(z_1,z_2)
=
\sum_{q=1}^{21}
\Gamma_{\mathrm{upper}}(r,q)
z_1^{\nu_{1,q}}
z_2^{\nu_{2,q}},
\]

where \((\nu_{1,q},\nu_{2,q})\) is read from the \(q\)-th entry of `monomial_index.csv`. The lower polynomial is evaluated analogously using `Gamma_lower.csv`.

Only the first

\[
N_f-1=15
\]

components of each endpoint family are independently fitted by least squares. To preserve the unit-sum constraint exactly, the last component is reconstructed according to Eq. (33):

\[
\hat f_{\sigma,16}(z)
=
1-
\sum_{r=1}^{15}
\hat f_{\sigma,r}(z),
\qquad
\sigma\in\{u,d\}.
\]

Consequently,

\[
\sum_{i=1}^{4}\sum_{j=1}^{4}
\hat f_{\sigma,ij}(z)=1,
\]

and the corresponding residuals satisfy the zero-sum relation given in Eqs. (34)–(35).

The complete coefficient matrices provided in this repository include the reconstructed 16th component. Therefore, all 16 endpoint polynomials can be evaluated directly from `Gamma_upper.csv` and `Gamma_lower.csv`.

For each endpoint family, 15 polynomials are independently estimated and each polynomial contains 21 coefficients. Thus, the total number of independently estimated coefficients is

\[
15\times21\times2=630.
\]

---

## 6. Step 4: Lifted Polynomial Vertices

The polynomial representation is converted into the lifted form of Eqs. (37)–(38).

For \(\eta=5\) and \(p=2\), define

\[
z'=
[
z_1,z_1^2,z_1^3,z_1^4,z_1^5,
z_2,z_2^2,z_2^3,z_2^4,z_2^5
]^T.
\]

The lifted dimension is therefore

\[
\eta p=10.
\]

Fifteen uniformly spaced partition nodes are used along each premise variable. Thus, each direction contains 14 intervals and the complete premise domain is divided into

\[
N_\Omega=14\times14=196
\]

local sub-domains.

For each local sub-domain \(\Omega_\mu\), determine the lower and upper bounds of every lifted coordinate \(z_k^\beta\). Selecting the lower or upper bound of each of the 10 lifted coordinates generates

\[
2^{\eta p}=2^{10}=1024
\]

lifted vertices per local sub-domain.

At each lifted vertex, evaluate the upper and lower polynomial endpoint values using the coefficient matrices supplied in this repository. These values correspond to

\[
\hat f'_{u,ij}
\left(
z'_{\mu,i_{1k}\cdots i_{\eta k}}
\right)
\]

and

\[
\hat f'_{d,ij}
\left(
z'_{\mu,i_{1k}\cdots i_{\eta k}}
\right)
\]

in Eq. (38).

No additional polynomial fitting is performed in the local sub-domains. The same global fifth-order polynomial representation is used over the entire premise domain.

---

## 7. Step 5: Approximation-Residual Bounds

The exact endpoint function and its polynomial approximation satisfy Eq. (39):

\[
\Delta f_{\sigma,ij}(z)
=
f_{\sigma,ij}(z)
-
\hat f_{\sigma,ij}(z).
\]

For each continuous local sub-domain \(\Omega_\mu\), constant lower and upper residual bounds are provided such that

\[
\underline{\Delta f}_{\sigma,ij,\mu}
\le
\Delta f_{\sigma,ij}(z)
\le
\overline{\Delta f}_{\sigma,ij,\mu},
\qquad
z\in\Omega_\mu.
\]

These bounds correspond to Eq. (40) of the manuscript and are stored in

`residual_bounds.csv`.

Each of the 196 local sub-domains was further divided into

\[
6\times6
\]

continuous interval boxes for residual certification. These additional boxes are used only to obtain continuous-domain residual bounds; they do not introduce additional polynomial fitting regions or additional LMI sub-domains.

For each row of `residual_bounds.csv`:

- `endpoint` specifies the upper or lower envelope;
- `region_1` and `region_2` specify the local premise-domain region;
- `z1_lower`, `z1_upper`, `z2_lower`, and `z2_upper` give its boundaries;
- `rule_i` and `rule_j` identify \(f_{ij}\);
- `residual_lower` gives
  \(\underline{\Delta f}_{\sigma,ij,\mu}\);
- `residual_upper` gives
  \(\overline{\Delta f}_{\sigma,ij,\mu}\).

Thus, Eq. (41) can be reconstructed as

\[
f_{\sigma,ij}(z)
=
\hat f'_{\sigma,ij}(z')
+
\Delta f_{\sigma,ij}(z),
\]

where the first term is handled through the finite lifted vertices and the second term is handled through the supplied continuous-domain residual bounds.

---

## 8. Step 6: Robust \(H_\infty\) LMI Synthesis

The supplied polynomial coefficients and residual bounds are used directly in the robust \(H_\infty\) endpoint LMIs of Theorem 2.

For every

\[
\mu=1,\ldots,N_\Omega,
\]

every one of the

\[
2^{\eta p}=1024
\]

lifted vertices, and both endpoint families
\(\sigma\in\{u,d\}\):

1. evaluate
   \(\hat f'_{\sigma,ij}\)
   at the corresponding lifted vertex using
   `Gamma_upper.csv` or `Gamma_lower.csv`;

2. obtain
   \(\underline{\Delta f}_{\sigma,ij,\mu}\)
   and
   \(\overline{\Delta f}_{\sigma,ij,\mu}\)
   from `residual_bounds.csv`;

3. construct the robust local matrix
   \(\Theta_{ij}\)
   using the USV model and the \(5\%\) parameter-uncertainty setting in Section IV-A; and

4. substitute these quantities into Eqs. (56)–(60) of Theorem 2.

The synthesis variables are

\[
N,\quad M_j,\quad T_{ij},\quad W,\quad
\gamma,\quad\epsilon.
\]

The reported results were obtained using YALMIP under MATLAB R2023b with the MOSEK solver, with \(\gamma\) minimized subject to the LMIs.

After solving the LMIs, the feedback gains are recovered from

\[
K_j=M_jN^{-1},
\qquad j=1,\ldots,4.
\]

For the fifth-order representation with \(\delta=0.05\), the reported minimum performance index is

\[
\gamma_{\min}=0.0020644.
\]

The resulting feedback gains should reproduce the values reported in Section IV-B of the manuscript.

---

## 9. Numerical Reproduction Checkpoints

The following values may be used to verify that the supplementary data have been interpreted consistently with the manuscript:

| Quantity | Value |
|---|---:|
| Number of fuzzy rules \(l\) | 4 |
| Number of augmented weights \(N_f=l^2\) | 16 |
| Number of premise variables \(p\) | 2 |
| Polynomial order \(\eta\) | 5 |
| Monomials per polynomial | 21 |
| Fitting samples \(N_s\) | 400 |
| Independently fitted components per endpoint | 15 |
| Independently estimated coefficients | 630 |
| Local sub-domains \(N_\Omega\) | 196 |
| Residual-certification boxes per local sub-domain | \(6\times6\) |
| Lifted dimension \(\eta p\) | 10 |
| Lifted vertices per local sub-domain | 1024 |
| Envelope-width parameter \(\delta\) | 0.05 |
| Maximum fitting error | \(2.252379\times10^{-3}\) |
| Maximum certified residual bound | \(7.03184323\times10^{-2}\) |
| Minimum \(H_\infty\) index | \(0.0020644\) |

Small numerical differences may occur because of solver tolerances and numerical precision.

---

## 10. Scope of the Repository

This repository is intended to provide the numerical data required to reproduce the finite-dimensional representation used in the manuscript. In particular, the supplied fitting samples, polynomial coefficients, monomial indices, and certified residual bounds allow the endpoint functions in Eqs. (36)–(41) to be reconstructed and substituted into the robust \(H_\infty\) LMIs in Theorem 2.

The implementation source code for envelope generation, polynomial fitting, residual certification, LMI construction, and the experimental USV controller is not included in this repository.
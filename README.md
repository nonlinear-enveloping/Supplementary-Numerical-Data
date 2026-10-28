# Supplementary Numerical Data

Companion dataset for **“Nonlinear Enveloping for Membership-Function-Dependent Synthesis of T-S Fuzzy Systems”**.

## Overview

This repository provides the numerical data supporting the polynomial representation and robust $`H_\infty`$ controller synthesis for the unmanned surface vehicle (USV) example in the accompanying manuscript.

The dataset corresponds to the theoretical procedure in Section III and the USV example in Section V. It includes sampled augmented envelope functions, complete fifth-order polynomial coefficients, the associated monomial indices, certified local approximation-residual bounds, and numerical validation statistics.

The sections below describe the data layout and its use in the synthesis procedure. Unless otherwise stated, section, equation, and theorem references refer to the manuscript. The repository contains numerical data only; implementation code and model matrices are outside its scope.

## Contents

- [Dataset Configuration](#dataset-configuration)
- [Repository Contents](#repository-contents)
- [USV Model and Membership Functions](#usv-model-and-membership-functions)
- [Radial Envelope Construction](#radial-envelope-construction)
- [Polynomial Representation and Coefficient Data](#polynomial-representation-and-coefficient-data)
- [Lifted Polynomial Representation](#lifted-polynomial-representation)
- [Approximation Residual Bounds](#approximation-residual-bounds)
- [Controller Synthesis](#controller-synthesis)
- [Reproducibility Checks](#reproducibility-checks)
- [Scope and Limitations](#scope-and-limitations)

<!-- BEGIN CURRENT CONFIGURATION -->
## Dataset Configuration

| Parameter | Value |
| :--- | ---: |
| Premise anchor $`z_0`$ | $`(0.8,0.2)`$ |
| Membership anchor $`h_0`$ | $`[0,0,1,0]^T`$ |
| Number of fuzzy rules $`l`$ | 4 |
| Number of premise variables $`p`$ | 2 |
| Envelope-width parameter $`\delta`$ | 0.05 |
| Polynomial order $`\eta`$ | 5 |
| Fitting samples $`N_s`$ | 400 |
| Augmented components $`N_f=l^2`$ | 16 |
| Monomials per polynomial | 21 |
| Independently fitted components per endpoint | 15 |
| Independently estimated coefficients | 630 |
| Lifted dimension $`\eta p`$ | 10 |
| Local sub-domains $`N_\Omega`$ | 196 |
| Certification boxes per sub-domain | $`6\times6`$ |
| Lifted vertices per sub-domain | 1024 |

The data were exported from `usv_data.mat`, `fitted_usv_data.mat`, and `usv_error_bounds.mat` in the originating calculation environment. These source files are not included in this repository. The anchor and envelope-width parameter are also recorded in [fitting_summary.csv](fitting_summary.csv) and [fitted_coefficients.mat](fitted_coefficients.mat).

<!-- END CURRENT CONFIGURATION -->

## Repository Contents

| File | Description | Data layout |
| :--- | :--- | :--- |
| [Gamma_upper.csv](Gamma_upper.csv) | Polynomial coefficients of the augmented upper-envelope functions $`\hat f_{u,ij}(z)`$. | 16 components; 21 coefficient columns and 5 metadata columns. |
| [Gamma_lower.csv](Gamma_lower.csv) | Polynomial coefficients of the augmented lower-envelope functions $`\hat f_{d,ij}(z)`$. | 16 components; 21 coefficient columns and 5 metadata columns. |
| [monomial_index.csv](monomial_index.csv) | Mapping from coefficient columns to the multi-index $`\nu=(\nu_1,\nu_2)`$. | 21 monomials. |
| [fitted_coefficients.mat](fitted_coefficients.mat) | MATLAB-format coefficient matrices and associated numerical metadata. | Binary MAT-file. |
| [fitting_samples.csv](fitting_samples.csv) | Sampled augmented upper and lower envelope functions used for least-squares fitting. | 400 samples; 2 premise-variable columns and 32 envelope-value columns. |
| [residual_bounds.csv](residual_bounds.csv) | Certified lower and upper residual bounds over the continuous local sub-domains. | 6,272 records: 196 sub-domains, 16 components, and 2 endpoint families. |
| [fitting_summary.csv](fitting_summary.csv) | Fitting, residual-bound, and normalization-validation statistics. | 26 metric–value pairs. |

All CSV files include a header row. Record counts in the table exclude headers.

## USV Model and Membership Functions

### State and Premise Variables

The USV state vector is

```math
x=[u,v,r,x,y,\psi]^T,
```

where $`u`$, $`v`$, and $`r`$ are the surge velocity, sway velocity, and yaw rate, respectively, and $`x`$, $`y`$, and $`\psi`$ denote the north position, east position, and yaw angle.

The two premise variables used for the T-S fuzzy model are

```math
z_1=u=x_1\in[0.8,2.8], \qquad
z_2=r=x_3\in[-0.2,0.2].
```

### Membership Functions and Rule Ordering

The four normalized membership functions used in the USV example are

```math
h_1(z_1,z_2)
=(0.5z_1-0.4)(0.5+2.5z_2),
```

```math
h_2(z_1,z_2)
=(0.5z_1-0.4)(0.5-2.5z_2),
```

```math
h_3(z_1,z_2)
=(1.4-0.5z_1)(0.5+2.5z_2),
```

```math
h_4(z_1,z_2)
=(1.4-0.5z_1)(0.5-2.5z_2).
```

They satisfy

```math
h_i(z)\ge 0,\qquad
\sum_{i=1}^{4}h_i(z)=1
```

over the identified premise-variable domain.

The corresponding premise-domain vertices are ordered as

```math
(2.8,0.2),\quad
(2.8,-0.2),\quad
(0.8,0.2),\quad
(0.8,-0.2),
```

for Rules 1–4, respectively.

### Nominal Model Parameters

The identified hydrodynamic parameters used to construct the nominal T-S model are

```math
m_{11}=82.8813,\qquad
m_{22}=83.4000,\qquad
m_{33}=9.8470,
```

```math
d_{11}=-25.2160,\qquad
d_{11q}=33.9944,\qquad
d_{22}=66.1440,\qquad
d_{33}=10.1492.
```

The model matrices are specified in the USV example of the manuscript. This dataset was exported from the nominal USV calculations. The membership-function samples, polynomial coefficients, and residual bounds can also be used with the uncertainty matrices of the robust case. Neither the model matrices nor the uncertainty matrices are included in this numerical-data package.

## Radial Envelope Construction

### Original Membership-Function Space

Following Section III-A, the anchor point used in the USV example is

```math
h_0=h(0.8,0.2)
=
[0,0,1,0]^T.
```

For an arbitrary $`z=(z_1,z_2)`$, define

```math
\Lambda(z)=\|h(z)-h_0\|_2
```

and, for $`\Lambda(z)>0`$,

```math
e(z)=\frac{h(z)-h_0}{\Lambda(z)}.
```

The upper and lower radial envelopes are defined by

```math
h_u(z)=h(z)+d_e(z)e(z),
```

```math
h_d(z)=h(z)-d_e(z)e(z),
```

corresponding to Eqs. (8)–(9) of the manuscript.

For the USV example,

```math
\delta=0.05
```

and

```math
d_e(z)
=
\delta\Phi(\vartheta(z))d_{\max}(z),
```

where $`d_{\max}(z)`$ is defined by Eq. (21). The shape function in Eq. (23) becomes

```math
\Phi(z_1,z_2)
=
\sin\left(
\frac{z_1-0.8}{2.0}\pi
\right)
\sin\left(
\frac{z_2+0.2}{0.4}\pi
\right).
```

This construction preserves the unit-sum and nonnegativity constraints of the probability simplex.

### Augmented Membership-Function Space

After applying the parallel distributed compensation (PDC) controller, the product weighting functions are defined according to Eq. (30) as

```math
f_{ij}(z)=h_i(z)h_j(z),
\qquad i,j=1,\ldots,4.
```

Hence,

```math
N_f=l^2=16.
```

The augmented components are stored in row-major order,

```math
f_{11},f_{12},f_{13},f_{14},
f_{21},\ldots,f_{44}.
```

Equivalently, the row index $`r`$ of component $`f_{ij}`$ is

```math
r=4(i-1)+j.
```

The same radial construction described above is applied directly to the 16-dimensional augmented weighting vector $`f(z)`$, as described in Section III-B, yielding

```math
f_{u,ij}(z),\qquad
f_{d,ij}(z).
```

The file `fitting_samples.csv` contains the values of these 16 upper and 16 lower augmented envelope functions at the $`20\times20`$ uniformly distributed fitting grid,

```math
N_s=400.
```

These data can be used to independently verify the polynomial fitting described below.

## Polynomial Representation and Coefficient Data

### Polynomial Basis

Following Eq. (36), each augmented endpoint function is represented by a complete multivariate polynomial

```math
\hat f_{\sigma,ij}(z)
=
\sum_{\nu\in\mathcal L_{\eta}}
\Gamma_{\sigma,ij,\nu}
z_1^{\nu_1}z_2^{\nu_2},
\qquad
\sigma\in\{u,d\},
```

where

```math
\mathcal L_{\eta}
=
\left\{
(\nu_1,\nu_2):
\nu_1+\nu_2\le\eta
\right\}.
```

For the reported USV result,

```math
\eta=5.
```

Therefore, each complete fifth-order bivariate polynomial contains

```math
\frac{(\eta+1)(\eta+2)}{2}
=
21
```

monomial coefficients.

### Coefficient Layout and Evaluation

The exponent pair $`(\nu_1,\nu_2)`$ associated with each coefficient column is provided in [monomial_index.csv](monomial_index.csv). The upper and lower coefficient matrices are provided in [Gamma_upper.csv](Gamma_upper.csv) and [Gamma_lower.csv](Gamma_lower.csv), respectively.

Each coefficient CSV contains five metadata columns (`component_index`, `rule_i`, `rule_j`, `component`, and `status`), followed by the 21 coefficient columns. Use the order specified by `coefficient_index` and `coefficient_name` in `monomial_index.csv` when assembling the numerical coefficient matrix.

For example, if row $`r`$ corresponds to $`f_{ij}`$, its upper polynomial is reconstructed as

```math
\hat f_{u,ij}(z_1,z_2)
=
\sum_{q=1}^{21}
\Gamma_{\mathrm{upper}}(r,q)
z_1^{\nu_{1,q}}
z_2^{\nu_{2,q}},
```

where $`(\nu_{1,q},\nu_{2,q})`$ is read from the $`q`$-th entry of `monomial_index.csv`. The lower polynomial is evaluated analogously using `Gamma_lower.csv`.

### Unit-Sum Reconstruction

Only the first

```math
N_f-1=15
```

components of each endpoint family are independently fitted by least squares. To preserve the unit-sum constraint exactly, the last component is reconstructed according to Eq. (33):

```math
\hat f_{\sigma,16}(z)
=
1-
\sum_{r=1}^{15}
\hat f_{\sigma,r}(z),
\qquad
\sigma\in\{u,d\}.
```

Consequently,

```math
\sum_{i=1}^{4}\sum_{j=1}^{4}
\hat f_{\sigma,ij}(z)=1,
```

and the corresponding residuals satisfy the zero-sum relation given in Eqs. (34)–(35).

The complete coefficient matrices provided in this repository include the reconstructed 16th component. Therefore, all 16 endpoint polynomials can be evaluated directly from `Gamma_upper.csv` and `Gamma_lower.csv`.

For each endpoint family, 15 polynomials are independently estimated and each polynomial contains 21 coefficients. Thus, the total number of independently estimated coefficients is

```math
15\times21\times2=630.
```

## Lifted Polynomial Representation

### Lifted Coordinates

The polynomial representation is converted into the lifted form of Eqs. (37)–(38).

For $`\eta=5`$ and $`p=2`$, define

```math
z'=
[
z_1,z_1^2,z_1^3,z_1^4,z_1^5,
z_2,z_2^2,z_2^3,z_2^4,z_2^5
]^T.
```

The lifted dimension is therefore

```math
\eta p=10.
```

### Local Partition and Vertex Construction

Fifteen uniformly spaced partition nodes are used along each premise variable. Thus, each direction contains 14 intervals and the complete premise domain is divided into

```math
N_\Omega=14\times14=196
```

local sub-domains.

For each local sub-domain $`\Omega_\mu`$, determine the lower and upper bounds of every lifted coordinate $`z_k^\beta`$. Selecting the lower or upper bound of each of the 10 lifted coordinates generates

```math
2^{\eta p}=2^{10}=1024
```

lifted vertices per local sub-domain.

At each lifted vertex, evaluate the upper and lower polynomial endpoint values using the coefficient matrices supplied in this repository. These values correspond to

```math
\hat f'_{u,ij}
\left(
z'_{\mu,i_{1k}\cdots i_{\eta k}}
\right)
```

and

```math
\hat f'_{d,ij}
\left(
z'_{\mu,i_{1k}\cdots i_{\eta k}}
\right)
```

in Eq. (38).

No additional polynomial fitting is performed in the local sub-domains. The same global fifth-order polynomial representation is used over the entire premise domain.

## Approximation Residual Bounds

### Certified Local Bounds

The exact endpoint function and its polynomial approximation satisfy Eq. (39):

```math
\Delta f_{\sigma,ij}(z)
=
f_{\sigma,ij}(z)
-
\hat f_{\sigma,ij}(z).
```

For each continuous local sub-domain $`\Omega_\mu`$, constant lower and upper residual bounds are provided such that

```math
\underline{\Delta} f_{\sigma,ij,\mu}
\le
\Delta f_{\sigma,ij}(z)
\le
\overline{\Delta} f_{\sigma,ij,\mu},
\qquad
z\in\Omega_\mu.
```

These bounds correspond to Eq. (40) of the manuscript and are stored in [residual_bounds.csv](residual_bounds.csv).

Each of the 196 local sub-domains was further divided into

```math
6\times6
```

continuous interval boxes for residual certification. These additional boxes are used only to obtain continuous-domain residual bounds; they do not introduce additional polynomial fitting regions or additional LMI sub-domains.

### Residual Data Fields

Each row of `residual_bounds.csv` describes one component of one endpoint family over one local sub-domain.

| Field | Description |
| :--- | :--- |
| `endpoint` | Endpoint family: `upper` or `lower`. |
| `region_1`, `region_2` | Local region indices along the two premise variables. |
| `z1_lower`, `z1_upper` | Lower and upper boundaries along $`z_1`$. |
| `z2_lower`, `z2_upper` | Lower and upper boundaries along $`z_2`$. |
| `comp_idx` | Augmented component index. |
| `ri`, `rj` | Rule indices $`i`$ and $`j`$ identifying $`f_{ij}`$. |
| `residual_lower` | Certified lower bound $`\underline{\Delta} f_{\sigma,ij,\mu}`$. |
| `residual_upper` | Certified upper bound $`\overline{\Delta} f_{\sigma,ij,\mu}`$. |

Thus, Eq. (41) can be reconstructed as

```math
f_{\sigma,ij}(z)
=
\hat f'_{\sigma,ij}(z')
+
\Delta f_{\sigma,ij}(z),
```

where the first term is handled through the finite lifted vertices and the second term is handled through the supplied continuous-domain residual bounds.

## Controller Synthesis

The supplied polynomial coefficients and residual bounds can be substituted into the endpoint linear matrix inequalities (LMIs) of Theorem 2 for robust $`H_\infty`$ synthesis, using the nominal or uncertain USV model as appropriate. The reference performance value below corresponds to the nominal USV calculation used for this dataset.

### LMI Construction

For every

```math
\mu=1,\ldots,N_\Omega,
```

every one of the

```math
2^{\eta p}=1024
```

lifted vertices, and both endpoint families
$`\sigma\in\{u,d\}`$:

1. Evaluate $`\hat f'_{\sigma,ij}`$ at the corresponding lifted vertex using `Gamma_upper.csv` or `Gamma_lower.csv`.
2. Retrieve $`\underline{\Delta} f_{\sigma,ij,\mu}`$ and $`\overline{\Delta} f_{\sigma,ij,\mu}`$ from `residual_bounds.csv`.
3. Construct the robust local matrix $`\Theta_{ij}`$ using the chosen USV model, including the corresponding uncertainty matrices when evaluating the robust case.
4. Substitute these quantities into Eqs. (56)–(60) of Theorem 2.

The synthesis variables are

```math
N,\quad M_j,\quad T_{ij},\quad W,\quad
\gamma,\quad\epsilon.
```

### Solver Configuration and Reference Result

The reported results were obtained using the following computational environment, with $`\gamma`$ minimized subject to the LMIs:

| Component | Software |
| :--- | :--- |
| Numerical environment | MATLAB R2023b |
| Optimization modeling | YALMIP |
| Solver | MOSEK |

After solving the LMIs, the feedback gains are recovered from

```math
K_j=M_jN^{-1},
\qquad j=1,\ldots,4.
```

For the fifth-order representation with $`\delta=0.05`$, the reported minimum performance index is

```math
\gamma_{\min}=0.0020644.
```

This reference value is for the nominal model comparison in the manuscript. The feedback gains and solver output are not included in this numerical-data package.

## Reproducibility Checks

The following checks help verify that the files have been interpreted consistently with the manuscript:

1. **Coefficient dimensions:** each endpoint family has 16 component rows and 21 numerical coefficient columns. Exclude the five metadata columns when assembling the coefficient matrices.
2. **Component ordering:** use the row-major convention defined above and the supplied component and rule indices.
3. **Fitting samples:** evaluate the polynomials at the 400 premise points in `fitting_samples.csv` and compare against the corresponding `fu_ij` and `fd_ij` columns.
4. **Unit-sum constraint:** verify that the reconstructed 16 endpoint polynomials sum to one for each endpoint family, up to numerical precision.
5. **Residual bounds:** match each of the 6,272 records to its endpoint family, local sub-domain, and augmented component before constructing the LMIs.

The numerical reference values are:

| Quantity | Reference value |
| :--- | ---: |
| Maximum absolute fitting error | $`2.252379234304\times10^{-3}`$ |
| Maximum absolute certified residual bound | $`7.031843234100\times10^{-2}`$ |
| Reference nominal $`H_\infty`$ index | $`0.0020644`$ |

The fitting and residual-bound statistics are recorded in [fitting_summary.csv](fitting_summary.csv). The nominal performance index is the synthesis reference described above.

Small numerical differences may occur because of solver tolerances and numerical precision.

## Scope and Limitations

This repository is intended to provide the numerical data required to reproduce the finite-dimensional representation used in the manuscript. In particular, the supplied fitting samples, polynomial coefficients, monomial indices, and certified residual bounds allow the endpoint functions in Eqs. (36)–(41) to be reconstructed and substituted into the robust $`H_\infty`$ LMIs in Theorem 2.

Reproducing the controller synthesis additionally requires the model matrices and LMI formulation specified in the manuscript, together with a compatible optimization environment. This repository does not include the nominal or uncertainty matrices, feedback gains, solver output, or implementation source code for envelope generation, polynomial fitting, residual certification, LMI construction, and the experimental USV controller.

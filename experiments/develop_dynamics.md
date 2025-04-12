
SE(3)における離散ダイナミクス
$$
\begin{align}
R_{k+1}&=R_kF_k\\
p_{k+1}&=p_k+hR_kv_k\\
Mv_{k+1}&= F_k^\top Mv_k+h\mathcal{U}_{k+1}+hf_k\\
J\Omega_{k+1}&=F_k^\top J\Omega_{k}+hMv_{k+1}\times v_{k+1}
+h\mathcal{M}_{k+1}+h\tau_k
\end{align}
$$
ただし
$$
\begin{align}
\mathcal{U}(p,R)&=-R^\top\frac{\partial U}{\partial p}(p,R)\\
\mathcal{M}(p,R)^\times&=\frac{\partial U}{\partial R}^\top R-R^\top \frac{\partial U}{\partial R}\\
U&=mgp_z 
\end{align}
$$
また一般的な近似として
$$
F_k \simeq \exp(h\Omega_k^\times) \simeq I+h\Omega_k^\times
$$
を用いる．
$f, \tau$を制御入力とし，$p_{k+1}$を$u_k=(f_k, \tau_k)$についての線形な関数として表すと
並進運動について
$$
\begin{align}
p_{k+1}&=p_k+hR_kv_k\\
Mv_{k+1}&= F_k^\top Mv_k+h\mathcal{U}_{k+1}+hf_k\\
&= Mv_k+h(Mv_k\times\Omega_k-mgR^\top e_z+f)\\
\end{align}
$$
回転運動について
$$
\begin{align}
R_{k+1}&=R_kF_k\simeq R_kF_{k+1}\\
Mv_{k+1}&= F_k^\top Mv_k+h\mathcal{U}_{k+1}+hf_k\\
&\simeq F_{k+1}^\top Mv_k+h\mathcal{U}_{k+1}+hf_k
\end{align}
$$
の仮定を認めると並進運動について
$$
\begin{align}
J\Omega_{k+1}&=F_k^\top J\Omega_{k}+\underbrace{hMv_{k+1}\times v_{k+1}}_{\simeq0}
+\underbrace{h\mathcal{M}_{k+1}}_{=0}+h\tau_k\\
&=J\Omega_k+\underbrace{hJ\Omega_k\times\Omega_k}_{\simeq0}+h\tau_k\\
R_{k+1}&=R_{k}+hR_{k}\Omega_{k+1}^\times\\
&=R_{k}+hR_{k}[\Omega_k+hJ^{-1}\tau_k]_\times\\
&=R_k+hR_k\Omega^\times_k+h^2R_{k}[J^{-1}\tau_k]_\times\\
\end{align}
$$
回転運動について(結局使っていない．)
$$
\begin{align}
v_{k+1}&= M^{-1}(F_{k+1}^\top Mv_k+h\mathcal{U}_{k+1}+hf_k)\\
&= v_k+h(v_k\times\Omega_{k+1}-gR^\top e_z+M^{-1}f)\\
&=v_k+h(v_k\times(\Omega_{k}+hJ^{-1}\tau_k)-gR^\top e_z+M^{-1}f)\\
&=v_k+h(v_k\times\Omega_{k}-gR^\top e_z+M^{-1}f)+h^2(v_k\times J^{-1}\tau_k)
\end{align}
$$
よってここから
$$
\begin{align}
p_{k+1}&=p_k+hR_{k+1}v_{k+1}\\
&=p_k+h(R_k+hR_k\Omega_{k+1}^\times)(v_k+h(v_k\times \Omega_{k}-gR_k^\top e_z+R_kM^{-1}fe_z))\\
&=p_k+h(R_k+hR_k\Omega^\times_k+h^2[J^{-1}\tau_k]_\times)(v_k+h(v_k\times \Omega_{k}-gR_k^\top e_z+R_kM^{-1}fe_z))\\
&=p_k+hR_kv_k+h^2(-ge_z+R_kM^{-1}fe_z)+h^3[J^{-1}\tau_k]_\times v_k+\mathcal{O}(h^4)
\end{align}
$$
ただしドローンの力学モデルに合わせるため
$$
\begin{align}
f_k&\mapsto R_kfe_z, f_k\in \mathbb{R}\\
u_k&=\begin{bmatrix}
f_k\\
\tau_k
\end{bmatrix}\in\mathbb{R}^4
\end{align}
$$
とした．
gptには$h^3\rightarrow h^2$として
$$
\begin{align}
p_{k+1}&=p_k+hR_{k+1}v_{k+1}\\
&=p_k+hR_kv_k+h^2\left[-ge_z+R_kM^{-1}fe_z+[J^{-1}\tau_k]_\times v_k\right]+\mathcal{O}(h^3)
\end{align}
$$
で良いのではと言われた(ほんまか？)

$$
\begin{align}
&\underbrace{p^d_{k}-p_k-hR_kv_k}_{e_k}-h^2(-ge_z+R_kM^{-1}fe_z)-h^3[J^{-1}\tau_k]_\times v_k\\
&=\underbrace{e_k+h^2ge_z}_{\tilde e_k}-h^2M^{-1}R_ke_zf+h^3v_k^\times J^{-1}\tau_k\\
&=\tilde e_k-h^2M^{-1}R_ke_zf+h^3v_k^\times J^{-1}\tau_k\\
&=\tilde e_k+A_ff_k+A_\tau\tau_k\\
&=\tilde e_k+
\begin{bmatrix}
A_f&A_\tau
\end{bmatrix}
\begin{bmatrix}
f_k\\
\tau_k
\end{bmatrix}\\
&=\tilde e_k+Au_k
\end{align}
$$
最小化したい目的関数は
$$
\begin{align}
J&=\frac{1}{2}\|\tilde e_k+A\|^2+\frac{1}{2}u_k^\top B u_k\\
&\propto \frac{1}{2}u_k^\top(A^\top A+B)u_k+(A^\top \tilde e_k)^\top u_k
\end{align}
$$
よって目標位置$p_k^d$に追従するためのQPは
$$
\begin{align}
\min_{u_k}&\: \frac{1}{2}u_k^\top(A^\top A+B)u_k+(A^\top \tilde e_k)^\top u_k\\
A&=\begin{bmatrix}
h^2M^{-1}R_ke_z&h^3v_k \times J^{-1}
\end{bmatrix}\in\mathbb{R}^{3\times 4}\\
\tilde e_k&=p^d_{k}-p_k-hR_kv_k+h^2ge_z
\end{align}
$$

### HOCBF
$$
h=\beta_l^{\top}(p_i)R_ie_z-\cos\Psi_\mathcal{F}
$$
とすると，安全制約の２階微分は
$$
\begin{align}
\ddot h &= \underbrace{-\frac{e_z^\top R^\top P_\beta R}{d}\dot v}_{\langle \mathrm{grad}_ph,\dot v\rangle}
+\underbrace{\frac{d}{dt}\left(-\frac{e_z^\top R^\top P_\beta R}{d}\right)v}_{\langle \mathrm{Hess}_ph[v],v\rangle
+\langle \mathrm{Hess}_ph[v],\Omega\rangle}\\
&\qquad 
+\underbrace{\left(\frac{P_\beta}{d}Rv\right)^\top R e_z^\times\Omega}_{\langle \mathrm{Hess}_Rh[\Omega],v\rangle}
\underbrace{-\beta^\top R\Omega^\times e_z^\times \Omega}_{\langle \mathrm{Hess}_Rh[\Omega],\Omega\rangle}
\underbrace{-\beta^\top Re_z^\times \dot\Omega}_{\langle \mathrm{grad}_Rh,\dot \Omega\rangle}\\
&=\langle \mathrm{grad}_ph,\dot v\rangle+\langle \mathrm{Hess}_ph[v],v\rangle
+\langle \mathrm{Hess}_ph[v],\Omega\rangle\\
&\qquad +\langle \mathrm{grad}_Rh,\dot \Omega\rangle
+\langle \mathrm{Hess}_Rh[\Omega],v\rangle
+\langle \mathrm{Hess}_Rh[\Omega],\Omega\rangle
\end{align}
$$
ただし，
$$
\begin{align}
\langle \mathrm{Hess}_ph[v],\Omega\rangle&=\langle \mathrm{Hess}_Rh[\Omega],v\rangle=v^\top R^\top \frac{P_\beta R e_z^\times}{d}\omega,\\
\langle \mathrm{Hess}_Rh[\Omega],\Omega\rangle&=\omega^\top[R^\top\beta]_\times e_z^\times\omega,\\
\mathrm{grad}_ph&=-\frac{e_z^\top R^\top P_\beta R}{d}\\
\mathrm{grad}_Rh&=-\beta^\top Re_z^\times\\
\end{align}
$$

$$
\begin{align}
\frac{d}{dt}\left(-\frac{e_z^\top R^\top P_\beta R}{d}\right)v
&=\underbrace{v^\top R^\top \frac{P_\beta R e_z^\times}{d}\Omega}_{\langle \mathrm{Hess}_ph[v],\Omega\rangle}
-\frac{z^\top\dot P_\beta}{d}Rv
-\frac{z^\top P_\beta}{d}R\Omega^\times v
-v^\top R^\top\frac{P_\beta z\beta^\top}{d^2}Rv\\
&=\langle \mathrm{Hess}_ph[v],\Omega\rangle\\
&\quad -\underbrace{v^\top R^\top\frac{\beta(z^\top P_\beta)+(z^\top \beta)P_\beta+P_\beta(Re_z)\beta^\top}{d^2}Rv-\frac{(Re_z)^\top P_\beta}{d}R\Omega^\times v}_{\langle \mathrm{Hess}_ph[v],v\rangle}\\
&=\langle \mathrm{Hess}_ph[v],v\rangle
+\langle \mathrm{Hess}_ph[v],\Omega\rangle\\
\end{align}
$$

よって２次系におけるHOCBFは
$$
\begin{align}
&\langle \mathrm{grad}_ph,\dot v\rangle+\langle \mathrm{Hess}_ph[v],v\rangle
+\langle \mathrm{Hess}_ph[v],\Omega\rangle +\langle \mathrm{grad}_Rh,\dot \Omega\rangle+\langle \mathrm{Hess}_Rh[\Omega],v\rangle+\langle \mathrm{Hess}_Rh[\Omega],\Omega\rangle\\
&+\gamma_1'(h)(\langle \mathrm{grad}_ph,v\rangle+\langle \mathrm{grad}_Rh,\Omega\rangle)+\gamma_2(\langle \mathrm{grad}_ph,v\rangle+\langle \mathrm{grad}_Rh,\Omega\rangle+\gamma_1(h(x)))\geq0
\end{align}
$$
簡単のため
$$
\gamma_1(h(x))=\gamma_2(h(x))=\gamma_0 h(x)
$$
とすると
$$
\begin{align}
&\langle \mathrm{grad}_ph,\dot v\rangle+\langle \mathrm{Hess}_ph[v],\Omega\rangle+\langle \mathrm{Hess}_ph[v],v\rangle
 +\langle \mathrm{grad}_Rh,\dot \Omega\rangle+\langle \mathrm{Hess}_Rh[\Omega],\Omega\rangle+\langle \mathrm{Hess}_Rh[\Omega],v\rangle\\
	&+2\gamma_0(\langle \mathrm{grad}_Rh,\Omega\rangle+\langle \mathrm{grad}_ph,v\rangle)+\gamma_0^2(h(x)))\geq0
\end{align}
$$
より
$$
\begin{align}
&\underbrace{-\frac{e_z^\top R^\top P_\beta R}{d}\dot v}_{\langle \mathrm{grad}_ph,\dot v\rangle}
+\underbrace{v^\top R^\top \frac{P_\beta R e_z^\times}{d}\Omega}_{\langle \mathrm{Hess}_ph[v],\Omega\rangle}\\
&\underbrace{-v^\top R^\top\frac{\beta(z^\top P_\beta)+(z^\top \beta)P_\beta+P_\beta(Re_z)\beta^\top}{d^2}Rv-\frac{(Re_z)^\top P_\beta}{d}R\Omega^\times v}_{\langle \mathrm{Hess}_ph[v],v\rangle}\\
&\underbrace{-\beta^\top Re_z^\times \dot\Omega}_{\langle \mathrm{grad}_Rh,\dot \Omega\rangle}
\underbrace{-\beta^\top R\Omega^\times e_z^\times \Omega}_{\langle \mathrm{Hess}_Rh[\Omega],\Omega\rangle}
+\underbrace{\left(\frac{P_\beta}{d}Rv\right)^\top R e_z^\times\Omega}_{\langle \mathrm{Hess}_Rh[\Omega],v\rangle}\\
&+2\gamma_0(\underbrace{-\beta^\top Re_z^\times\Omega}_{\langle \mathrm{grad}_Rh,\Omega\rangle}\underbrace{-\frac{e_z^\top R^\top P_\beta R}{d}v}_{\langle \mathrm{grad}_ph,v\rangle})+\gamma_0^2(\beta_l^{\top}Re_z-\cos\Psi_\mathcal{F})\geq0
\end{align}
$$
$\dot v, \dot \Omega$を書き下すと
$$
\dot v\simeq J^{-1}, \quad\dot \Omega \simeq Mv^\times\Omega-mgR^\top e_z+f
$$
として
$$
\begin{align}
&\underbrace{-\beta^\top Re_z^\times J^{-1}\tau}_{\langle \mathrm{grad}_Rh,\dot \Omega\rangle}
\underbrace{-\frac{e_z^\top R^\top P_\beta R}{d}(Mv^\times\Omega-mgR^\top e_z+f)}_{\langle \mathrm{grad}_ph,\dot v\rangle}\\
&+\underbrace{v^\top R^\top \frac{P_\beta R e_z^\times}{d}\Omega}_{\langle \mathrm{Hess}_ph[v],\Omega\rangle}\\
&\underbrace{-v^\top R^\top\frac{\beta(z^\top P_\beta)+(z^\top \beta)P_\beta+P_\beta(Re_z)\beta^\top}{d^2}Rv-\frac{(Re_z)^\top P_\beta}{d}R\Omega^\times v}_{\langle \mathrm{Hess}_ph[v],v\rangle}\\
&
\underbrace{-\beta^\top R\Omega^\times e_z^\times \Omega}_{\langle \mathrm{Hess}_Rh[\Omega],\Omega\rangle}
+\underbrace{\left(\frac{P_\beta}{d}Rv\right)^\top R e_z^\times\Omega}_{\langle \mathrm{Hess}_Rh[\Omega],v\rangle}\\
&+2\gamma_0(\underbrace{-\beta^\top Re_z^\times\Omega}_{\langle \mathrm{grad}_Rh,\Omega\rangle}\underbrace{-\frac{e_z^\top R^\top P_\beta R}{d}v}_{\langle \mathrm{grad}_ph,v\rangle})+\gamma_0^2(\beta_l^{\top}Re_z-\cos\Psi_\mathcal{F})\geq0
\end{align}
$$

となる．
したがって、制約付きQPは以下のように定式化される：
$$
\begin{align}
\min_{u_k}&\: \frac{1}{2}u_k^\top(A^\top A+B)u_k+(A^\top \tilde e_k)^\top u_k\\
\mathrm{s.t.}&\: Cu_k \geq b
\end{align}
$$

ここで、
$$
\begin{align}
u_k &= \begin{bmatrix} f_k \\ \tau_k \end{bmatrix} \in \mathbb{R}^4\\
A &= \begin{bmatrix} h^2M^{-1}R_ke_z & h^3v_k \times J^{-1} \end{bmatrix} \in \mathbb{R}^{3\times 4}\\
\tilde e_k &= p^d_{k}-p_k-hR_kv_k+h^2ge_z\\
C &= \begin{bmatrix} -\frac{e_z^\top R^\top P_\beta R}{d} & -\beta^\top Re_z^\times J^{-1} \end{bmatrix} \in \mathbb{R}^{1\times 4}\\
b &= -\langle \mathrm{Hess}_ph[v],\Omega\rangle - \langle \mathrm{Hess}_ph[v],v\rangle - \langle \mathrm{Hess}_Rh[\Omega],v\rangle - \langle \mathrm{Hess}_Rh[\Omega],\Omega\rangle\\
&\quad - 2\gamma_0(\langle \mathrm{grad}_Rh,\Omega\rangle + \langle \mathrm{grad}_ph,v\rangle) - \gamma_0^2(\beta_l^{\top}Re_z-\cos\Psi_\mathcal{F})\\
&\quad + \frac{e_z^\top R^\top P_\beta R}{d}(Mv^\times\Omega-mgR^\top e_z)
\end{align}
$$

この制約付きQPを解くことで、目標位置への追従と安全制約の両方を満たす制御入力$u_k$を得ることができる。
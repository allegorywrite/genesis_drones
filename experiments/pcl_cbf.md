## 複数の特徴点を追従する安全制約
複数の特徴点を追従することを考える場合，安全集合を
$$
\begin{align}
&\sum_{l\in\mathcal{L}}\Psi_{i}^l -m\geq0\\\tag{14}
&\Psi_{ij}^l = \left\{ \begin{array}{ll}
0 & \mathrm{if} \quad  \beta_l^{\top}(p_i)R_ie_c-\cos\Psi_\mathcal{F} < 0\\
1 & \mathrm{if}  \quad  \beta_l^{\top}(p_i)R_ie_c-\cos\Psi_\mathcal{F} \geq 0
\end{array} \right.
\end{align}
$$
等とすると$\Psi_{ij}^l$が微分不可能になるため，
$q_l\in \mathcal{L}$によってエージェント$i$における推定が成り立っている確率を
$$
\phi_{i}^l= P(p_i,R_i,q^l)\tag{15}
$$
とすると新しい安全集合を
$$
B_{i}=1-q-\prod_{l\in\mathcal{L}}(1-\phi_{i}^l)\tag{16}
$$
とすることで環境内の特徴点$q_l\in \mathcal{L}$によってエージェント$i$における推定が達成される確率を$q$以上に制限することができる．ここで一旦
$$
\begin{align}
P(p_i,R_i,q^l) &= \left\{ \begin{array}{ll}\tag{17}
P_i^l& \mathrm{if}  \quad  q_l\in\mathcal{C}_i \\
0 & \mathrm{if}  \quad  q_l\in \mathcal{L}\setminus\mathcal{C}_i
\end{array} \right.\\
\mathrm{where} \quad P_i^l &= \frac{\beta_l^\top(p_i) R_i e_c -\cos\Psi_\mathcal{F} }{1-\cos\Psi_\mathcal{F}}
\end{align}
$$
とすると以下の安全制約を構成できる．
$$
B_{i}>0 \quad \forall i \in \mathcal{A}\tag{18}
$$
$$
\begin{align}
B_{i}&=1-q-\eta_{i}\\\tag{19}
\eta_{i}&=\prod_{l\in\mathcal{L}}(1-\phi_{i}^l)\\
\end{align}
$$
$$
\begin{align}
\dot B_{i}&=-\dot\eta_{i}\\\tag{20}
&=-\frac{d}{dt}\prod_{l\in\mathcal{L}}(1-\phi_{i}^l)\\
&=\sum_{l\in \mathcal{L}}(\prod_{k\neq l}(1-\phi_{i}^k))\dot \phi^l_{i}\\
\dot \phi^l_{i} &= \left\{ \begin{array}{ll}
\dot P_i^l & \mathrm{if}  \quad  q_l\in\mathcal{C}_i\\
0 & \mathrm{if}  \quad  q_l\in \mathcal{L}\setminus\mathcal{C}_i
\end{array} \right.\\
\end{align}
$$
$P_i^l$の微分は以下のように計算できる．
$$
\begin{align}\tag{21}
P_i^l &= \frac{\beta_l^\top(p_i) R_i e_c -\cos\Psi_\mathcal{F} }{1-\cos\Psi_\mathcal{F}}\\
\dot P_i^l&=\langle \mathrm{grad}\:P_i^l, \xi_W\rangle\\
&= \left\langle 
\begin{bmatrix}\mathrm{grad}_R\:P_i^l\\\mathrm{grad}_p\:P_i^l
\end{bmatrix},
\begin{bmatrix}\omega\\v
\end{bmatrix}
\right\rangle\\
\mathrm{grad}_p\:P_i^l &= \frac{1}{1-\cos \psi_\mathcal{F}}(-\frac{e_c^\top R_i^\top P_{\beta_l}}{d})\\
\mathrm{grad}_R\:P_i^l &= \frac{1}{1-\cos \psi_\mathcal{F}}(-\beta_l^\top(p_i) R_i [e_c]_\times)\\
P_{\beta_l} &= I-\beta\beta^\top
\end{align}
$$
### 速度入力及び角速度入力について分解しない場合
$$
\begin{align}\tag{22}
\sum_{l\in \mathcal{L}\subset\mathcal{C}_i }(\prod_{k\neq l}(1-\phi_{i}^k)) \langle \mathrm{grad}_R\:P_i^l,\omega_i\rangle&+\sum_{l\in \mathcal{L}\subset\mathcal{C}_i }(\prod_{k\neq l}(1-\phi_{i}^k)) \langle \mathrm{grad}_p\:P_i^l,v_i\rangle
\\&> - \gamma_0 (1-q-\prod_{l\in\mathcal{L}}(1-\phi_{i}^k))
\end{align}
$$
よってQPは以下のようになる
$$
\begin{align}\tag{23}
 \min_{\xi_i, c_1}\:(p^d_{i}-p_{i,{k+1}}-hR_{i,k}v_{i,k})^\top Q_1 & (p^d_{i}-p_{i,{k+1}}-hR_{i,k}v_{i,k})
+ 
\begin{bmatrix}
\omega_{i,k}\\v_{i,k}
\end{bmatrix}^\top Q_2
\begin{bmatrix}
\omega_{i,k}\\v_{i,k}
\end{bmatrix}\\
\mathrm{s.t.}
\sum_{l\in \mathcal{L}\subset\mathcal{C}_i }(\prod_{k\neq l}(1-\phi_{i}^k)) \frac{\beta_l^\top(p_i) R_i [e_c]_\times}{1-\cos \psi_\mathcal{F}}\omega_i&+\sum_{l\in \mathcal{L}\subset\mathcal{C}_i }(\prod_{k\neq l}(1-\phi_{i}^k)) \frac{e_c^\top R_i^\top P_{\beta_l}}{(1-\cos \psi_\mathcal{F})d}v_i
\\&< \gamma_0 (1-q-\prod_{l\in\mathcal{L}}(1-\phi_{i}^k))\\
\end{align}
$$
一般的なQPの形式に変更すると以下のように表せる．
$$
\begin{align}\tag{24}
&\min_{\xi_i}\:J_i\\
J_i &= \frac{1}{2}\xi_{i,k}^\top H_i \xi_{i,k} + f_i^T  \xi_{i,k}\\
 \xi_{i,k} &= \begin{bmatrix}
\omega_{i,k}\\v_{i,k}
\end{bmatrix},
H_i = 2\begin{bmatrix}
Q_{2,\omega}&Q_{2,\omega v}\\ Q^\top_{2,\omega}&Q_{2,v}+h^2R_{i,k}^\top Q_1R_{i,k}
\end{bmatrix},\\
f_i &= \begin{bmatrix}
0\\-2hR_i^\top Q_1 e_i
\end{bmatrix}, e_i = p^d_{i,k}-p_{i,k}\\
\mathrm{s.t.} &\quad
\begin{bmatrix}
\sum_{l\in \mathcal{L}\subset\mathcal{C}_i}(\prod_{k\neq l}(1-\phi_{i}^k)) \frac{\beta_l^\top(p_i) R_i [e_c]_\times}{1-\cos \psi_\mathcal{F}}
\\
\sum_{l\in \mathcal{L}\subset\mathcal{C}_i}(\prod_{k\neq l}(1-\phi_{i}^k))\frac{e_c^\top R_i^\top P_{\beta_l}}{(1-\cos \psi_\mathcal{F})d}
\end{bmatrix}^\top\xi_{i,k}\leq
\gamma_0(1-q-\prod_{l\in \mathcal{L}\subset\mathcal{C}_i}(1-\phi_{i}^k))\\
\end{align}
$$

### 速度入力及び角速度入力について分解する場合
$$
\begin{align}\tag{25}
\sum_{l\in \mathcal{L}\subset\mathcal{C}_i }(\prod_{k\neq l}(1-\phi_{i}^k)) \langle \mathrm{grad}_R\:P_i^l,\omega_i\rangle
&> -c_1 \gamma (1-q-\prod_{l\in\mathcal{L}}(1-\phi_{i}^k))\\
\sum_{l\in \mathcal{L}\subset\mathcal{C}_i }(\prod_{k\neq l}(1-\phi_{i}^k)) \langle \mathrm{grad}_p\:P_i^l,v_i\rangle
&> -c_2 \gamma (1-q-\prod_{l\in\mathcal{L}}(1-\phi_{i}^k))\\
c_1 + c_2 &= \gamma_0 > 0\\
\frac{\gamma_0}{1-d_m}&<c_1<\frac{\gamma_0}{1-d_M}\\
\end{align}
$$
よってQPは以下のようになる．
$$
\begin{align}\tag{26}
 \min_{\xi_i, c_1}\:(p^d_{i}-p_{i,{k+1}}-hR_{i,k}v_{i,k})^\top Q_1 & (p^d_{i}-p_{i,{k+1}}-hR_{i,k}v_{i,k})
+ 
\begin{bmatrix}
\omega_{i,k}\\v_{i,k}
\end{bmatrix}^\top Q_2
\begin{bmatrix}
\omega_{i,k}\\v_{i,k}
\end{bmatrix}\\
\mathrm{s.t.}
\sum_{l\in \mathcal{L}\subset\mathcal{C}_i}(\prod_{k\neq l}(1-\phi_{i}^k)) \frac{\beta_l^\top(p_i) R_i [e_c]_\times}{1-\cos \psi_\mathcal{F}}\omega_i&< c_1 \gamma (1-q-\prod_{l\in\mathcal{L}}(1-\phi_{i}^k))\\
\sum_{l\in \mathcal{L}\subset\mathcal{C}_i}(\prod_{k\neq l}(1-\phi_{i}^k))\frac{e_c^\top R_i^\top P_{\beta_l}}{(1-\cos \psi_\mathcal{F})d}v_i&< c_2 \gamma (1-q-\prod_{l\in\mathcal{L}}(1-\phi_{i}^k))\\
c_1 + c_2 &= \gamma_0 > 0\\
\frac{\gamma_0}{1-d_m}&<c_1<\frac{\gamma_0}{1-d_M}
\end{align}
$$
一般的なQPの形式に変更すると以下のように表せる．
$$
\begin{align}\tag{27}
&\min_{\hat \xi_i}\:J_i\\
J_i &= \frac{1}{2}\hat \xi_{i,k}^\top H_i\hat \xi_{i,k} + f_i^T \hat \xi_{i,k}\\
\hat \xi_{i,k} &= \begin{bmatrix}
\omega_{i,k}\\v_{i,k}\\c_1
\end{bmatrix},
H_i = 2\begin{bmatrix}
Q_{2,\omega}&Q_{2,\omega v}&0\\ Q^\top_{2,\omega}&Q_{2,v}+h^2R_{i,k}^\top Q_1R_{i,k}&0\\0&0&0
\end{bmatrix},\\
f_i &= \begin{bmatrix}
0\\-2hR_i^\top Q_1 e_i\\0
\end{bmatrix}, e_i = p^d_{i,k}-p_{i,k}\\
\mathrm{s.t.} &\quad
\begin{bmatrix}
\alpha_\omega&0&-\gamma\\0&\alpha_v&\gamma\\
0&0&1\\
0&0&-1
\end{bmatrix}
\hat\xi_{i,k}\leq
\begin{bmatrix}
0\\\gamma\\\frac{1}{1-d_m}\\ \frac{1}{1-d_M}
\end{bmatrix}\gamma_0\\
\end{align}
$$
なおCBFによる制約式の係数は
$$
\begin{align}\tag{28}
\alpha_\omega&=\sum_{l\in \mathcal{L}\subset\mathcal{C}_i}(\prod_{k\neq l}(1-\phi_{i}^k)) \frac{\beta_l^\top(p_i) R_i [e_c]_\times}{1-\cos \psi_\mathcal{F}}\in\mathbb{R}^3\\
\alpha_v&=\sum_{l\in \mathcal{L}\subset\mathcal{C}_i}(\prod_{k\neq l}(1-\phi_{i}^k))\frac{e_c^\top R_i^\top P_{\beta_l}}{(1-\cos \psi_\mathcal{F})d}\in\mathbb{R}^3\\
\gamma &= 1-q-\prod_{l\in \mathcal{L}\subset\mathcal{C}_i}(1-\phi_{i}^k)\in\mathbb{R}
\end{align}
$$

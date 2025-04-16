## 複数のエージェントが共通の特徴点を追従する安全制約
エージェント位置を$T_i=(p_i, R_i)\in \mathrm{SE}(3), i\in\mathcal{A}$，環境内の特徴点を$q_l\in \mathcal{L}$とする．
エージェントiの観測している特徴点集合を$\mathcal{C}_i$とする．
$q_l\in\mathcal{C}_i \cap \mathcal{C}_j$である条件は
$$
(\beta_l^{\top}(p_i)R_ie_c-\cos\Psi_\mathcal{F})(\beta_l^{\top}(p_j)R_je_c-\cos\Psi_\mathcal{F})>0\tag{29}
$$

安全集合を
$$
\begin{align}\tag{30}
&\sum_{l\in\mathcal{L}}\Psi_{ij}^l -m\geq0\\
&\Psi_{ij}^l = \left\{ \begin{array}{ll}
0 & \mathrm{if} \quad  (\beta_l^{\top}(p_i)R_ie_c-\cos\Psi_\mathcal{F})(\beta_l^{\top}(p_j)R_je_c-\cos\Psi_\mathcal{F}) < 0\\
1 & \mathrm{if}  \quad  (\beta_l^{\top}(p_i)R_ie_c-\cos\Psi_\mathcal{F})(\beta_l^{\top}(p_j)R_je_c-\cos\Psi_\mathcal{F}) \geq 0
\end{array} \right.
\end{align}
$$
等とすると$\Psi_{ij}^l$が微分不可能になるため，
$q_l\in \mathcal{L}$によってエッジ$(i,j)\in \mathcal{E}$における推定が成り立っている確率を
$$
\phi_{ij}^l= P(p_i,R_i,p_j,R_j,q^l)\tag{31}
$$
とすると新しい安全集合を
$$
B_{ij}=1-q-\prod_{l\in\mathcal{L}}(1-\phi_{ij}^l)\tag{32}
$$

とすることで環境内の特徴点$q_l\in \mathcal{L}$によってエッジ$(i,j)\in \mathcal{E}$における推定が達成される確率を$q$以上に制限することができる．ここで一旦
$$
\begin{align}\tag{33}
P(p_i,R_i,p_j,R_j,q^l) &= \left\{ \begin{array}{ll}
P_i^lP_j^l & \mathrm{if}  \quad  q_l\in\mathcal{C}_i \cap \mathcal{C}_j\\
0 & \mathrm{if}  \quad  q_l\in \mathcal{L}\setminus\mathcal{C}_i \cap \mathcal{C}_j
\end{array} \right.\\
\mathrm{where} \quad P_i^l &= \frac{\beta_l^\top(p_i) R_i e_c -\cos\Psi_\mathcal{F} }{1-\cos\Psi_\mathcal{F}}\\
\end{align}
$$
とすると，２つの安全制約を構成できる
1. 指定されたエッジ上でのCoFOVを保証するCCBF
$$
B_{ij}>0 \quad \forall (i,j) \in \mathcal{E}\tag{34}
$$
2. エッジ$\mathcal{E}_i$上のいずれかのエージェントとのCoFOVを保証するCCBF
$$
\begin{align}\tag{35}
&B_i=\sum_{j\in \mathcal{E}_i}\Psi(B_{ij})>0\quad \forall i\in \mathcal{A}\\
&\Psi(c) = \left\{ \begin{array}{ll}
c & \mathrm{if} \quad  c > 0\\
0 & \mathrm{if}  \quad  c \leq 0
\end{array} \right.
\end{align}
$$
CBF Revisited
$$
\begin{align}\tag{36}
B_{ij}&=1-q-\eta_{ij}\\
\eta_{ij}&=\prod_{l\in\mathcal{L}}(1-\phi_{ij}^l)\\
\end{align}
$$
$$
\begin{align}\tag{37}
\dot B_{ij}&=-\dot\eta_{ij}\\
&=-\frac{d}{dt}\prod_{l\in\mathcal{L}}(1-\phi_{ij}^l)\\
&=\sum_{l\in \mathcal{L}}(\prod_{k\neq l}(1-\phi_{ij}^k))\dot \phi^l_{ij}\\
\dot \phi^l_{ij} &= \left\{ \begin{array}{ll}
\dot P_i^lP_j^l+P_i^l\dot P_j^l & \mathrm{if}  \quad  q_l\in\mathcal{C}_i \cap \mathcal{C}_j\\
0 & \mathrm{if}  \quad  q_l\in \mathcal{L}\setminus\mathcal{C}_i \cap \mathcal{C}_j
\end{array} \right.\\
\end{align}
$$

エージェントごとの制御入力について分解すると，
$$
\begin{align}\tag{38}
\dot B_{ij}&=-\dot\eta_{ij}\\
&=-\frac{d}{dt}\prod_{l\in\mathcal{L}}(1-\phi_{ij}^l)\\
&=\sum_{l\in \mathcal{L}}(\prod_{k\neq l}(1-\phi_{ij}^k))\dot \phi^l_{ij}\\
&= \sum_{l\in \mathcal{L}\subset\mathcal{C}_i \cap \mathcal{C}_j}(\prod_{k\neq l}(1-\phi_{ij}^k))P_j^l\dot P_i^l+\sum_{l\in \mathcal{L}\subset\mathcal{C}_i \cap \mathcal{C}_j}(\prod_{k\neq l}(1-\phi_{ij}^k))P_i^l\dot P_j^l
\end{align}
$$

### 速度入力及び角速度入力について分解しない場合
$$
\begin{align}\tag{39}
\sum_{l\in \mathcal{L}\subset\mathcal{C}_i \cap \mathcal{C}_j}(\prod_{k\neq l}(1-\phi_{ij}^k))P_j^l \langle \mathrm{grad}_R\:P_i^l,\omega_i\rangle&+\sum_{l\in \mathcal{L}\subset\mathcal{C}_i \cap \mathcal{C}_j}(\prod_{k\neq l}(1-\phi_{ij}^k))P_i^l \langle \mathrm{grad}_R\:P_j^l,\omega_j \rangle \\
+\sum_{l\in \mathcal{L}\subset\mathcal{C}_i \cap \mathcal{C}_j}(\prod_{k\neq l}(1-\phi_{ij}^k))P_j^l \langle \mathrm{grad}_p\:P_i^l,v_i\rangle&+\sum_{l\in \mathcal{L}\subset\mathcal{C}_i \cap \mathcal{C}_j}(\prod_{k\neq l}(1-\phi_{ij}^k))P_i^l \langle \mathrm{grad}_p\:P_j^l, v_j\rangle\\
&> -\gamma_0 (1-q-\prod_{l\in\mathcal{L}}(1-\phi_{ij}^k))
\end{align}
$$
よってQPは以下のようになる．
$$
\begin{align}\tag{40}
 \min_{\xi_i, \xi_j}\:\sum_{i,j}(p^d_{i}-p_{i,{k+1}}-hR_{i,k}v_{i,k})^\top Q_1 & (p^d_{i}-p_{i,{k+1}}-hR_{i,k}v_{i,k})
+ 
\begin{bmatrix}
\omega_{i,k}\\v_{i,k}
\end{bmatrix}^\top Q_2
\begin{bmatrix}
\omega_{i,k}\\v_{i,k}
\end{bmatrix}\\
\mathrm{s.t.}
\sum_{l\in \mathcal{L}\subset\mathcal{C}_i \cap \mathcal{C}_j}(\prod_{k\neq l}(1-\phi_{ij}^k)) \frac{P_j^l\beta_l^\top(p_i) R_i [e_c]_\times}{1-\cos \psi_\mathcal{F}}\omega_i&+\sum_{l\in \mathcal{L}\subset\mathcal{C}_i \cap \mathcal{C}_j}(\prod_{k\neq l}(1-\phi_{ij}^k)) 
\frac{P_i^l\beta_l^\top(p_j) R_j [e_c]_\times}{1-\cos \psi_\mathcal{F}}\omega_j
\\
+\sum_{l\in \mathcal{L}\subset\mathcal{C}_i \cap \mathcal{C}_j}(\prod_{k\neq l}(1-\phi_{ij}^k))\frac{P_j^le_c^\top R_i^\top P_{\beta_l}}{(1-\cos \psi_\mathcal{F})d}R_iv_i&
+\sum_{l\in \mathcal{L}\subset\mathcal{C}_i \cap \mathcal{C}_j}(\prod_{k\neq l}(1-\phi_{ij}^k))\frac{P_i^le_c^\top R_j^\top P_{\beta_l}}{(1-\cos \psi_\mathcal{F})d}R_jv_j\\
&< \gamma_0 (1-q-\prod_{l\in\mathcal{L}}(1-\phi_{ij}^k))
\end{align}
$$

一般的なQPの形式に変更すると以下のように表せる．
$$
\begin{align}\tag{41}
&\min_{\xi_i, \xi_j}\:\sum_{i,j}J_i\\
J_i &= \frac{1}{2}\xi_{i,k}^\top H_i\xi_{i,k} + f_i^T\xi_{i,k}\\
\xi_{i,k} &= \begin{bmatrix}
\omega_{i,k}\\v_{i,k}
\end{bmatrix},
H_i = 2\begin{bmatrix}
Q_{2,\omega}&Q_{2,\omega v}\\ Q^\top_{2,\omega}&Q_{2,v}+h^2R_{i,k}^\top Q_1R_{i,k}
\end{bmatrix},\\
f_i &= \begin{bmatrix}
0\\-2hR_i^\top Q_1 e_i
\end{bmatrix}, e_i = p^d_{i,k}-p_{i,k}\\
\mathrm{s.t.} &\quad\begin{bmatrix}
\alpha_\omega&\alpha_v&\beta_\omega&\beta_v
\end{bmatrix}
\begin{bmatrix}
\omega_{i,k}\\v_{i,k}\\\omega_{j,k}\\v_{j,k}
\end{bmatrix}\leq
\gamma_0\gamma
\end{align}
$$

もしくは目的関数$J_i, J_j$を合成して
$$
\begin{align}\tag{42}
&\min_{\xi_k} \quad \frac{1}{2}\xi_k^\top \begin{bmatrix} H_i & 0 \\ 0 & H_j \end{bmatrix}\xi_k + \begin{bmatrix} f_i\\ f_j \end{bmatrix}^\top\xi_k,\\
\xi_k &= \begin{bmatrix}
\xi_{i,k}\\\xi_{j,k}
\end{bmatrix},
H_i = 2\begin{bmatrix}
Q_{2,\omega}&Q_{2,\omega v}\\ Q^\top_{2,\omega}&Q_{2,v}+h^2R_{i,k}^\top Q_1R_{i,k}
\end{bmatrix},
f_i = \begin{bmatrix}
0\\-2hR_i^\top Q_1 e_i
\end{bmatrix}\\
\mathrm{s.t.} &\quad\begin{bmatrix}
\alpha_\omega&\alpha_v&\beta_\omega&\beta_v
\end{bmatrix}
\begin{bmatrix}
\omega_{i,k}\\v_{i,k}\\\omega_{j,k}\\v_{j,k}
\end{bmatrix}\leq
\gamma_0\gamma
\end{align}
$$
なおCBFによる制約式の係数は
$$
\begin{align}\tag{43}
\alpha_\omega&=\sum_{l\in \mathcal{L}\subset\mathcal{C}_i \cap \mathcal{C}_j}(\prod_{k\neq l}(1-\phi_{ij}^k)) \frac{P_j^l\beta_l^\top(p_i) R_i [e_c]_\times}{1-\cos \psi_\mathcal{F}}\in\mathbb{R}^3\\
\beta_\omega&=\sum_{l\in \mathcal{L}\subset\mathcal{C}_i \cap \mathcal{C}_j}(\prod_{k\neq l}(1-\phi_{ij}^k)) 
\frac{P_i^l\beta_l^\top(p_j) R_j [e_c]_\times}{1-\cos \psi_\mathcal{F}}\in\mathbb{R}^3\\
\alpha_v&=\sum_{l\in \mathcal{L}\subset\mathcal{C}_i \cap \mathcal{C}_j}(\prod_{k\neq l}(1-\phi_{ij}^k))\frac{P_j^le_c^\top R_i^\top P_{\beta_l}}{(1-\cos \psi_\mathcal{F})d}R_i\in\mathbb{R}^3\\
\beta_v&=\sum_{l\in \mathcal{L}\subset\mathcal{C}_i \cap \mathcal{C}_j}(\prod_{k\neq l}(1-\phi_{ij}^k))\frac{P_i^le_c^\top R_j^\top P_{\beta_l}}{(1-\cos \psi_\mathcal{F})d}R_j\in\mathbb{R}^3 \\
\gamma &= 1-q-\prod_{l\in \mathcal{L}\subset\mathcal{C}_i \cap \mathcal{C}_j}(1-\phi_{ij}^k)\in\mathbb{R}
\end{align}
$$

### 速度入力及び角速度入力について分解する場合
$$
\begin{align}\tag{39}
\sum_{l\in \mathcal{L}\subset\mathcal{C}_i \cap \mathcal{C}_j}(\prod_{k\neq l}(1-\phi_{ij}^k))P_j^l \langle \mathrm{grad}_R\:P_i^l,\omega_i\rangle&+\sum_{l\in \mathcal{L}\subset\mathcal{C}_i \cap \mathcal{C}_j}(\prod_{k\neq l}(1-\phi_{ij}^k))P_i^l \langle \mathrm{grad}_R\:P_j^l,\omega_j \rangle \\
&> -c_1 \gamma (1-q-\prod_{l\in\mathcal{L}}(1-\phi_{ij}^k))\\
\sum_{l\in \mathcal{L}\subset\mathcal{C}_i \cap \mathcal{C}_j}(\prod_{k\neq l}(1-\phi_{ij}^k))P_j^l \langle \mathrm{grad}_p\:P_i^l,v_i\rangle&+\sum_{l\in \mathcal{L}\subset\mathcal{C}_i \cap \mathcal{C}_j}(\prod_{k\neq l}(1-\phi_{ij}^k))P_i^l \langle \mathrm{grad}_p\:P_j^l, v_j\rangle\\
&> -c_2 \gamma (1-q-\prod_{l\in\mathcal{L}}(1-\phi_{ij}^k))\\
c_1 + c_2 &= \gamma_0 > 0
\end{align}
$$

よってQPは以下のようになる．
$$
\begin{align}\tag{40}
 \min_{\xi_i, \xi_j}\:\sum_{i,j}(p^d_{i}-p_{i,{k+1}}-hR_{i,k}v_{i,k})^\top Q_1 & (p^d_{i}-p_{i,{k+1}}-hR_{i,k}v_{i,k})
+ 
\begin{bmatrix}
\omega_{i,k}\\v_{i,k}
\end{bmatrix}^\top Q_2
\begin{bmatrix}
\omega_{i,k}\\v_{i,k}
\end{bmatrix}\\
\mathrm{s.t.}
\sum_{l\in \mathcal{L}\subset\mathcal{C}_i \cap \mathcal{C}_j}(\prod_{k\neq l}(1-\phi_{ij}^k)) \frac{P_j^l\beta_l^\top(p_i) R_i [e_c]_\times}{1-\cos \psi_\mathcal{F}}\omega_i&+\sum_{l\in \mathcal{L}\subset\mathcal{C}_i \cap \mathcal{C}_j}(\prod_{k\neq l}(1-\phi_{ij}^k)) 
\frac{P_i^l\beta_l^\top(p_j) R_j [e_c]_\times}{1-\cos \psi_\mathcal{F}}\omega_j
\\
&< c_1 \gamma (1-q-\prod_{l\in\mathcal{L}}(1-\phi_{ij}^k))\\
\sum_{l\in \mathcal{L}\subset\mathcal{C}_i \cap \mathcal{C}_j}(\prod_{k\neq l}(1-\phi_{ij}^k))\frac{P_j^le_c^\top R_i^\top P_{\beta_l}}{(1-\cos \psi_\mathcal{F})d}v_i&
+\sum_{l\in \mathcal{L}\subset\mathcal{C}_i \cap \mathcal{C}_j}(\prod_{k\neq l}(1-\phi_{ij}^k))\frac{P_i^le_c^\top R_j^\top P_{\beta_l}}{(1-\cos \psi_\mathcal{F})d}v_j\\
&< c_2 \gamma (1-q-\prod_{l\in\mathcal{L}}(1-\phi_{ij}^k))\\
c_1 + c_2 &= \gamma_0 > 0
\end{align}
$$

一般的なQPの形式に変更すると以下のように表せる．
$$
\begin{align}\tag{41}
&\min_{\xi_i, \xi_j}\:\sum_{i,j}J_i\\
J_i &= \frac{1}{2}\xi_{i,k}^\top H_i\xi_{i,k} + f_i^T\xi_{i,k}\\
\xi_{i,k} &= \begin{bmatrix}
\omega_{i,k}\\v_{i,k}
\end{bmatrix},
H_i = 2\begin{bmatrix}
Q_{2,\omega}&Q_{2,\omega v}\\ Q^\top_{2,\omega}&Q_{2,v}+h^2R_{i,k}^\top Q_1R_{i,k}
\end{bmatrix},\\
f_i &= \begin{bmatrix}
0\\-2hR_i^\top Q_1 e_i
\end{bmatrix}, e_i = p^d_{i,k}-p_{i,k}\\
\mathrm{s.t.} &\quad\begin{bmatrix}
\alpha_\omega&0&\beta_\omega&0\\0&\alpha_v&0&\beta_v
\end{bmatrix}
\begin{bmatrix}
\omega_{i,k}\\v_{i,k}\\\omega_{j,k}\\v_{j,k}
\end{bmatrix}\leq
\begin{bmatrix}
c_1\\c_2
\end{bmatrix}\gamma, \quad c_1+c_2 =\gamma_0 > 0\\
\end{align}
$$
もしくは目的関数$J_i, J_j$を合成して
$$
\begin{align}\tag{42}
&\min_{\xi_k} \quad \frac{1}{2}\xi_k^\top \begin{bmatrix} H_i & 0 \\ 0 & H_j \end{bmatrix}\xi_k + \begin{bmatrix} f_i\\ f_j \end{bmatrix}^\top\xi_k,\\
\xi_k &= \begin{bmatrix}
\xi_{i,k}\\\xi_{j,k}
\end{bmatrix},
H_i = 2\begin{bmatrix}
Q_{2,\omega}&Q_{2,\omega v}\\ Q^\top_{2,\omega}&Q_{2,v}+h^2R_{i,k}^\top Q_1R_{i,k}
\end{bmatrix},
f_i = \begin{bmatrix}
0\\-2hR_i^\top Q_1 e_i
\end{bmatrix}\\
\mathrm{s.t.} &\quad\begin{bmatrix}
\alpha_\omega&0&\beta_\omega&0\\0&\alpha_v&0&\beta_v
\end{bmatrix}
\xi_k\leq
\begin{bmatrix}
c_1\\c_2
\end{bmatrix}\gamma, \quad c_1+c_2 =\gamma_0 > 0\\
\end{align}
$$
なおCBFによる制約式の係数は
$$
\begin{align}\tag{43}
\alpha_\omega&=\sum_{l\in \mathcal{L}\subset\mathcal{C}_i \cap \mathcal{C}_j}(\prod_{k\neq l}(1-\phi_{ij}^k)) \frac{P_j^l\beta_l^\top(p_i) R_i [e_c]_\times}{1-\cos \psi_\mathcal{F}}\in\mathbb{R}^3\\
\beta_\omega&=\sum_{l\in \mathcal{L}\subset\mathcal{C}_i \cap \mathcal{C}_j}(\prod_{k\neq l}(1-\phi_{ij}^k)) 
\frac{P_i^l\beta_l^\top(p_j) R_j [e_c]_\times}{1-\cos \psi_\mathcal{F}}\in\mathbb{R}^3\\
\alpha_v&=\sum_{l\in \mathcal{L}\subset\mathcal{C}_i \cap \mathcal{C}_j}(\prod_{k\neq l}(1-\phi_{ij}^k))\frac{P_j^le_c^\top R_i^\top P_{\beta_l}}{(1-\cos \psi_\mathcal{F})d}\in\mathbb{R}^3\\
\beta_v&=\sum_{l\in \mathcal{L}\subset\mathcal{C}_i \cap \mathcal{C}_j}(\prod_{k\neq l}(1-\phi_{ij}^k))\frac{P_i^le_c^\top R_j^\top P_{\beta_l}}{(1-\cos \psi_\mathcal{F})d}\in\mathbb{R}^3\\
\gamma &= 1-q-\prod_{l\in \mathcal{L}\subset\mathcal{C}_i \cap \mathcal{C}_j}(1-\phi_{ij}^k)\in\mathbb{R}
\end{align}
$$

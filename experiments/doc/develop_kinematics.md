## 単一の特徴点を追従する安全制約

エージェント位置を$T_i=(p_i, R_i)\in \mathrm{SE}(3), i\in\mathcal{A}$，環境内の特徴点を$q_l\in \mathcal{L}$とする．
エージェントiの観測している特徴点集合を$\mathcal{C}_i$とする．
$q_l\in\mathcal{C}_i$である条件は
$$
\beta_l^{\top}(p_i)R_ie_c-\cos\Psi_\mathcal{F}>0\tag{1}
$$
ただし$\beta_l(p_i)=\frac{q_l-p_i}{\|q_l-p_i\|}$，$e_c=[0\:0\:1]$ （カメラ方向)，$\Psi_\mathcal{F}$はFoVとする．
安全集合をそのまま
$$
\begin{align}\tag{2}
B_{i}&=\beta_l^{\top}(p_i)R_ie_c-\cos\Psi_\mathcal{F}\\
\dot B_{i}&=\langle \mathrm{grad}_R\:B_i,\omega_i\rangle+ \langle \mathrm{grad}_p\:B_i,v_i\rangle\\
&=-\beta_l^\top(p_i) R_i [e_c]_\times\omega_i-\frac{e_c^\top R_i^\top P_{\beta_l}}{d_{i,l}}R_i v_i
\end{align}
$$
として安全制約は
$$\tag{3}
\beta_l^\top(p_i) R_i [e_c]_\times\omega_i+\frac{e_c^\top R_i^\top P_{\beta_l}}{d_{i,l}}R_iv_i\leq \gamma (\beta_l^{\top}(p_i)R_ie_c-\cos\Psi_\mathcal{F})
$$
<font color="red"> CBFにおけるv_iがワールド座標系なのを忘れていた．．．(修正済) </font>

ここで，$d_{i,l}={\|q_l-p_i\|}$，$P_{\beta_l} = I-\beta\beta^\top$ (投影行列)である．

### ドローンダイナミクス
ボディ座標系における速度入力$\xi^\wedge_B$によるSE(3)上の剛体運動式は
$$
\begin{align}\tag{4}
T&=\begin{bmatrix}
R&p\\
0&1
\end{bmatrix}\in\mathrm{SE}(3)\\
\quad \xi^\wedge_B&=\begin{bmatrix}
[\omega]_\times&v_b\\
0&0
\end{bmatrix}\in \mathfrak{se}(3)\\
\dot T &= T \xi^\wedge_B\\
\end{align}
$$
で示される．
世界座標系における速度入力$\xi^\wedge_W$による剛体運動式は
$$
\begin{align}\tag{5}
\dot T &= \xi^\wedge_W T \\
\xi_W^\wedge&=(\mathrm{Ad}_T\xi_B)^\wedge \\&= \left(\begin{bmatrix}
R&0\\
[p]_\times R&R
\end{bmatrix}
\begin{bmatrix}
\omega\\
v_b
\end{bmatrix}\right)^\wedge\\
&=
\begin{bmatrix}
R\omega\\
[p]_\times R\omega+Rv_b
\end{bmatrix}^\wedge\\
&=
\begin{bmatrix}
[R\omega]_\times&[p]_\times R\omega+v\\
0&0
\end{bmatrix}
\end{align}
$$
で変換できる．

ボディ座標系における運動方程式を離散化すると
$$
\begin{align}\tag{6}
T_{k+1}&\simeq T_k+\dot T_k \\
&\simeq T_k+h T_k\xi^\wedge_{B,k} \\
&= T_k
+h\begin{bmatrix}
R_k&p_k\\
0&1
\end{bmatrix}
\begin{bmatrix}
[\omega_k]_\times&v_k\\
0&0
\end{bmatrix}\\
\end{align}
$$
成分分解すると
$$
\begin{align}\tag{7}
R_{k+1}&\simeq  R_k\exp(h[\omega_k]_{\times})\\
&\simeq R_k(I+h[\omega_k]_{\times})\\
p_{k+1}&=hR_kv_k+p_k
\end{align}
$$
で計算できる．

CBF制約を満たしつつ，目標位置$p_i^d$に追従するためのQPは以下のようになる
$$
\begin{align}\tag{8}
 \min_{\xi_i, c_1}\:(p^d_{i}-p_{i,{k}}-hR_{i,k}v_{i,k})^\top Q_1 & (p^d_{i}-p_{i,{k}}-hR_{i,k}v_{i,k})
+ 
\begin{bmatrix}
\omega_{i,k}\\v_{i,k}
\end{bmatrix}^\top Q_2
\begin{bmatrix}
\omega_{i,k}\\v_{i,k}
\end{bmatrix}\\
\mathrm{s.t.}\quad
\beta_l^\top(p_i) R_i [e_c]_\times\omega_i&+\frac{e_c^\top R_i^\top P_{\beta_l}}{d_{i,l}}R_iv_i\leq \gamma (\beta_l^{\top}(p_i)R_ie_c-\cos\Psi_\mathcal{F})
\end{align}
$$
一般的な形式に変換すると
$$
\begin{align}\tag{9}
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
\beta_l^\top(p_i) R_i [e_c]_\times
\\
\frac{e_c^\top R_i^\top P_{\beta_l}}{d}R_i
\end{bmatrix}^\top\xi_{i,k}\leq
\gamma (\beta_l^{\top}(p_i)R_ie_c-\cos\Psi_\mathcal{F})\\
\end{align}
$$
と表せる．
![[Screenshot from 2025-04-09 13-27-18.png]]
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

これらから，複数の特徴点を追従するためのCBFは以下のようになる
$$
\begin{align}\tag{22}
\sum_{l\in \mathcal{L}\subset\mathcal{C}_i }(\prod_{k\neq l}(1-\phi_{i}^k)) \langle \mathrm{grad}_R\:P_i^l,\omega_i\rangle&+\sum_{l\in \mathcal{L}\subset\mathcal{C}_i }(\prod_{k\neq l}(1-\phi_{i}^k)) \langle \mathrm{grad}_p\:P_i^l,v_i\rangle
\\&> - \gamma_0 (1-q-\prod_{l\in\mathcal{L}}(1-\phi_{i}^k))
\end{align}
$$
よってQPは以下のようになる
$$
\begin{align}
 \min_{\xi_{i,k}}&\:(p^d_{i}-p_{i,{k+1}})^\top Q_1 (p^d_{i}-p_{i,{k+1}})
+ \xi_{i,k}^\top Q_2
\xi_{i,k}\\
&\mathrm{s.t.}
\sum_{l\in \mathcal{L}\subset\mathcal{C}_i }(\prod_{k\neq l}(1-P_i^l)) \langle 
\mathrm{grad}_T\:P_{i}^l,\xi_{i,k}\rangle\\
&\qquad > - \gamma_0 (1-q-\prod_{l\in\mathcal{L}}(1-P_i^l))\\
\end{align}
$$

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
\:H_i = 2\begin{bmatrix}
Q_{2,\omega}&Q_{2,\omega v}\\ Q^\top_{2,\omega}&Q_{2,v}+h^2R_{i,k}^\top Q_1R_{i,k}
\end{bmatrix},\\
f_i &= \begin{bmatrix}
0\\-2hR_i^\top Q_1 e_i
\end{bmatrix}, \:e_i = p^d_{i,k}-p_{i,k}\\
\mathrm{s.t.} &\quad
\begin{bmatrix}
\sum_{l\in \mathcal{L}\subset\mathcal{C}_i}(\prod_{k\neq l}(1-\phi_{i}^k)) \frac{\beta_l^\top(p_i) R_i [e_c]_\times}{1-\cos \psi_\mathcal{F}}
\\
\sum_{l\in \mathcal{L}\subset\mathcal{C}_i}(\prod_{k\neq l}(1-\phi_{i}^k))\frac{e_c^\top R_i^\top P_{\beta_l}}{(1-\cos \psi_\mathcal{F})d}R_i
\end{bmatrix}^\top\xi_{i,k}\leq
\gamma_0(1-q-\prod_{l\in \mathcal{L}\subset\mathcal{C}_i}(1-\phi_{i}^k))\\
\end{align}
$$
![[Screenshot from 2025-04-08 00-02-17.png]]
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

よってエージェント$i$と$j$が共通の特徴点を追従するためのCBFは以下のようになる．
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
![[Screenshot from 2025-04-09 02-18-18.png]]
## 複数のエージェントが分散的に共通の特徴点を追従する安全制約

不等式制約付き最適化問題を分散化する方法はいくつかあるが，とりあえず最も一般的だと感じる(勘)PDMMを最初に検討する．
### PDMM
なぜADMMでなくPDMM(主双対乗数法)なのかというと，主双対乗数法だと不等式制約を処理する場合にADMMのように等式制約にもっていくためのスラック変数修正などが必要なくなるかららしい？
```
The main difference between extended ADMM and PDMM is that no additional correction step is required in PDMM to handle the inequality constraints, resulting in significant faster convergence and lower computational complexity, as is demonstrated in Section VII.
```
参考：
https://arxiv.org/pdf/2309.12897#:~:text=In%20recent%20years%2C%20a%20number,cs.DC%5D%2017%20Feb%202024

IEQ-PDMM(inequality constraint primal-dual method of multipliers)を用いて上の式を分散化すると以下のようになる
$$
\begin{align}
\xi_i &= \underset{\xi_i}{\mathrm{argmin}} \:J_i(\xi_i)+z_{i|j}^\top A_{ij}\xi_i+\frac{c}{2}\|A_{ij}\xi_i-\frac{1}{2}\gamma_0\gamma\|^2\\
y_{i|j}&=z_{i|j}+2c(A_{ij}\xi_{ij}-\frac{1}{2}\gamma_0\gamma)\\
&\mathbf{node}_j\leftarrow \mathbf{node}_i(y_{i|j})\\
&\mathbf{if}\:y_{i|j}+y_{j|i}>0\\
&\qquad z_{i|j}=y_{j|i}\\
&\mathbf{else}\:\\
&\qquad z_{i|j}=-y_{i|j}\\
\end{align}
$$
ただし
$$
A_{ij}=\begin{bmatrix}
\alpha_\omega \\ \alpha_v
\end{bmatrix}, A_{ji}=\begin{bmatrix}
\beta_\omega \\ \beta_v
\end{bmatrix}
$$
QPの形式に表すと
$$
\begin{align}\tag{41}
&\min_{\xi_i}\:\hat J_i= \frac{1}{2}\xi_{i,k}^\top \hat H_i\xi_{i,k} + \hat f_i^\top \xi_{i,k}\\
\xi_{i,k} &= \begin{bmatrix}
\omega_{i,k}\\v_{i,k}
\end{bmatrix},
\hat H_i = \begin{bmatrix}
2Q_{2,\omega}+c\alpha_\omega^\top \alpha_\omega
&2Q_{2,\omega v}+c\alpha_\omega^\top \alpha_v\\
2Q^\top_{2,\omega}+c\alpha_v^\top \alpha_\omega 
&2Q_{2,v}+2h^2R_{i,k}^\top Q_1R_{i,k}+c\alpha_v^\top \alpha_v
\end{bmatrix},\\
\hat f_i &= \begin{bmatrix}
z_{i|j}\alpha_\omega^\top -\frac{c}{2}\gamma_0\gamma\alpha_\omega^\top
\\-2hR_i^\top Q_1 e_i+z_{i|j}\alpha_v^\top -\frac{c}{2}\gamma_0\gamma\alpha_v^\top
\end{bmatrix}, e_i = p^d_{i,k}-p_{i,k}
\end{align}
$$
![[Screenshot from 2025-04-09 13-24-49.png]]
一方で，この計算手法では制約式がすべてソフト制約化するため最適化中に元の制約式を満たすことを保証できない．
### CBF-induced QP
https://people.kth.se/~dimos/pdfs/DistributedCBF_LCSS_2022.pdf#:~:text=Abstract%E2%80%94%20In%20this%20work%2C%20we,Instead%2C%20to%20solve%20the%20quadratic

最適化中に元の制約式を満たすことを保証する？ADMMベースのCBF分散化手法
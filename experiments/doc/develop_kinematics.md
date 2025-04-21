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


### 自己位置推定のためのCBF

視野共有を保証するCBFにおいては，
$$
\begin{align}
&\Psi_{i}^l = \left\{ \begin{array}{ll}
0 & \mathrm{if} \quad  \beta_l^{\top}(p_i)R_ie_c-\cos\Psi_\mathcal{F} < 0\\
1 & \mathrm{if}  \quad  \beta_l^{\top}(p_i)R_ie_c-\cos\Psi_\mathcal{F} \geq 0
\end{array} \right.
\end{align}
$$
が微分不可能であることを避けるため
$q_l\in \mathcal{L}$によってエージェント$i$における推定が成り立っている確率を
$$
\phi_{i}^l= P(p_i,R_i,q^l)\tag{15}
$$
とした．本研究では簡単のため
$$
\begin{align}
\phi_{ij}^l &= \frac{\beta_l^\top(p_i) R_i e_c -\cos\Psi_\mathcal{F} }{1-\cos\Psi_\mathcal{F}}\frac{\beta_l^\top(p_j) R_j e_c -\cos\Psi_\mathcal{F} }{1-\cos\Psi_\mathcal{F}}
\end{align}
$$
という関数を使用しているが．$P^l_i$が$(p_i, R_i)\in \mathrm{SE(3)}$について微分可能であれば任意の確率関数を設計可能である．今後の目標として，自己位置推定における関数を下限制約することを目的として$\phi^l_{ij}$を設計することを考える．

各エージェント$i \in \mathcal{A}$に対して，観測モデルは
$$
\tilde{p}_i = \pi_i(q_l) + w_i
$$
と表される．ここで，
- $\pi_i(q_l)$はエージェント$i$における$q_l$の非線形な投影関数である．一般的なピンホールモデルの場合，画像座標$(u_i, v_i)$は内部パラメータ（焦点距離や主点）や$q_l$の空間的な位置$(X, Y, Z)$と関係する．
- $w_i$は白色ノイズであり，通常は独立同分布の正規分布$w_i \sim \mathcal{N}(0, \sigma_i^2 I)$（または各エージェントごとに異なる共分散行列を考慮する場合もある）と仮定される．

各観測ノイズが正規分布に従うことから，対数尤度関数は
$$
\mathcal{L}(q_l) = -\sum_i \frac{1}{2}(\tilde{p}_i - \pi_i(q_l))^T \Sigma_i^{-1}(\tilde{p}_i - \pi_i(q_l)) + \text{定数}
$$
となる．ここで，各$\Sigma_i$は$w_i$の共分散行列である．最尤推定量$\hat{q}_l$はこの対数尤度を最大化する解，または等価に
$$
\hat{q}_l = \operatorname*{arg\,min}_{q_l} \sum_i \| \tilde{p}_i - \pi_i(q_l) \|^2_{\Sigma_i^{-1}}
$$
を満たす$q_l$として求められる．

投影関数$\pi_i(q_l)$は一般に非線形であるため，解析的に誤差伝搬や不確かさを評価するには，$q_l$の推定値の周りで一階テイラー展開を用いる．

任意の推定点$q_0^l$周りで
   $$
   \pi_i(q_l) \approx \pi_i(q_0^l) + J_i (q_l - q_0^l)
   $$
と近似する．ここで，$J_i = \frac{\partial \pi_i}{\partial q_l} \Big|_{q_l=q_0^l}$はエージェント$i$における投影関数のヤコビアンである．
この近似により，観測式は
   $$
   \tilde{p}_i \approx \pi_i(q_0^l) + J_i (q_l - q_0^l) + w_i
   $$
と表せ，$q_l$に対する線形モデルが得られる．これにより，小さな誤差領域での推定誤差伝搬解析が可能となる．

最尤推定問題において，推定値の下界としてのCramér-Rao Lower Bound (CRLB)は，フィッシャー情報行列$\mathbf{I}(q_l)$を用いて記述される．線形化した場合，FIMは以下の形になる．
$$
\mathbf{I}(q_l) = \sum_i J_i^T \Sigma_i^{-1} J_i
$$
ここで，各エージェント$i$からの情報が加算されることにより，複数台のカメラを用いることでより高い情報量，すなわち低い推定誤差が期待できる．

CRLBより，任意の不偏推定量$\hat{q}_l$の共分散行列は次の下界を満たす：
$$
\operatorname{Cov}(\hat{q}_l) \ge \mathbf{I}(q_l)^{-1} = \left( \sum_i J_i^T \Sigma_i^{-1} J_i \right)^{-1}.
$$

特に，各エージェントのノイズが等方性（すべて$\Sigma_i = \sigma^2 I$とする場合），上式は
$$
\operatorname{Cov}(\hat{q}_l) \ge \sigma^2 \left( \sum_i J_i^T J_i \right)^{-1}
$$
となり，エージェント数が増えることにより各々の寄与が積み重なり，結果として推定の不確かさが低下する傾向が明確となる．
より詳細な解析として，解析的なヤコビアン
$$
J = \frac{f}{d}\,P_\beta, \quad P_\beta = I - \beta\,\beta^\top,
$$
（ここで$\beta = \frac{q_l-p}{\|q_l-p\|}$はエージェントから観測点$q_l$への単位方向ベクトル，すなわちbearing）を用いて，CRLBを評価することができる．

各エージェントについて，対象$q_l$の投影に関する解析的なヤコビアンは
$$
J_i = \frac{f}{d_i}\,P_{\beta_i}, \quad d_i = \|q_l-p_i\|, \quad P_{\beta_i} = I - \beta_i\,\beta_i^\top,
$$
と記述できる．ここで
- $f$は焦点距離，
- $d_i$は対象とエージェント$i$との距離，
- $\beta_i = \frac{q_l-p_i}{d_i}$はエージェント$i$におけるbearing，
- $P_{\beta_i}$は$\beta_i$に沿った成分を取り除く射影行列（すなわち，画像面上の変化にのみ寄与する部分）である．

同様に，エージェント$j$についても
$$
J_j = \frac{f}{d_j}\,P_{\beta_j}, \quad d_j = \|q_l-p_j\|, \quad \beta_j = \frac{q_l-p_j}{d_j}.
$$

それぞれのエージェントからのフィッシャー情報行列は
$$
I_i = J_i^\top J_i, \quad I_j = J_j^\top J_j,
$$
となる．ここで，射影行列$P_\beta$は対称かつ冪等（$P_\beta^2 = P_\beta$）なので，
$$
J_i^\top J_i = \left(\frac{f}{d_i}P_{\beta_i}\right)^\top \left(\frac{f}{d_i}P_{\beta_i}\right) = \frac{f^2}{d_i^2}\,P_{\beta_i}^\top P_{\beta_i} = \frac{f^2}{d_i^2}\,P_{\beta_i}.
$$
同様に，
$$
J_j^\top J_j = \frac{f^2}{d_j^2}\,P_{\beta_j}.
$$

したがって，2台のカメラからの合成情報は
$$
I_{\text{total}} = J_i^\top J_i + J_j^\top J_j = \frac{f^2}{d_i^2}\,P_{\beta_i} + \frac{f^2}{d_j^2}\,P_{\beta_j}.
$$

CRLBにより，無偏推定量$\hat{q}_l$の共分散行列は
$$
\operatorname{Cov}(q_l) \ge \sigma^2\,\frac{1}{f^2}\,\left[\frac{P_{\beta_i}}{d_i^2} + \frac{P_{\beta_j}}{d_j^2}\right]^{-1}.
$$
ここから，新しい自己位置推定のための確率関数を
$$
\begin{align}
\phi_{ij}^l &= E_{ij}^{l} =  \exp(-\frac{\sigma^2}{f^2}\,\mathrm{tr}\left[ \frac{P_{\beta_i}}{d_i^2} + \frac{P_{\beta_j}}{d_j^2}\right]^{-1})
\end{align}
$$
のように設計できる．

### 確率関数の時間微分

上記で設計した確率関数を制御に利用するためには，その時間微分を計算する必要がある．ここでは，確率関数の時間微分を段階的に導出する．
$$
M \triangleq \frac{P_{\beta_i}}{d_i^2}+\frac{P_{\beta_j}}{d_j^2}
$$
とおき，$M^{-1}$のトレースを $T$ と書くと
$$
T = \operatorname{tr}(M^{-1})
$$
となり，提案した関数はより簡潔に
$$
{E}_{ij}^e = \exp\Bigl(-\frac{\sigma^2}{f^2}\,T\Bigr)
$$
と表せる．$T = \operatorname{tr}(M^{-1})$ として，チェーンルールを適用すると
$$
\dot{{E}}_{ij}^l = -\frac{\sigma^2}{f^2}\,{P}_{ij}^l\,\dot{T}
$$
ここで
$$
\dot{T} = \frac{d}{dt}\operatorname{tr}(M^{-1}) =\operatorname{tr}\Bigl(\frac{d}{dt}(M^{-1})\Bigr) =-\operatorname{tr}\Bigl(M^{-1}\,\dot{M}\,M^{-1}\Bigr)
$$
これを上記の式に代入すると、
$$
\dot{{E}}_{ij}^l = -\frac{\sigma^2}{f^2}\,{E}_{ij}^l\,\Bigl[-\operatorname{tr}(M^{-1}\,\dot{M}\,M^{-1})\Bigr] =\frac{\sigma^2}{f^2}\,{E}_{ij}^l\,\operatorname{tr}(M^{-1}\,\dot{M}\,M^{-1})
$$
よって、
$$
\dot{E}_{ij}^l = \frac{\sigma^2}{f^2}\,{E}_{ij}^l\,\operatorname{tr}\Bigl(M^{-1}\,\dot{M}\,M^{-1}\Bigr)
$$
ここで、$M$ は各エージェント $i,j$ について
$$
M = \frac{P_{\beta_i}}{d_i^2} + \frac{P_{\beta_j}}{d_j^2}
$$
と定義されている。各項に対して微分を行う。一般に、スカラー関数 $d$ と行列 $P_\beta$ の組合せの微分は、
$$
\frac{d}{dt}\left(\frac{P_\beta}{d^2}\right) =\frac{1}{d^2}\dot{P}_\beta - \frac{2\dot{d}}{d^3}\,P_\beta
$$
ここで
$$
\dot{P}_\beta = \frac{P_\beta\,Rv}{d}\,\beta^\top + \beta\,\frac{R^\top v^\top\,P_\beta}{d}
$$
より、各エージェント $i$ について
$$
\frac{d}{dt}\left(\frac{P_{\beta_i}}{d_i^2}\right) =\frac{1}{d_i^2}\left(\frac{P_{\beta_i}\,R_iv_i}{d_i}\,\beta_i^\top + \beta_i\,\frac{R_i^\top v_i^\top\,P_{\beta_i}}{d_i}\right) -\frac{2(-\beta_i^\top R_i v_i)}{d_i^3}\,P_{\beta_i}
$$
すなわち、
$$
\frac{d}{dt}\left(\frac{P_{\beta_i}}{d_i^2}\right) =\frac{P_{\beta_i}\,R_iv_i\,\beta_i^\top + \beta_i\,R_i^\top v_i^\top\,P_{\beta_i}}{d_i^3} +\frac{2(\beta_i^\top R_iv_i)}{d_i^3}\,P_{\beta_i}
$$
ゆえに、
$$
\dot{M} = \frac{P_{\beta_i}\,R_iv_i\,\beta_i^\top + \beta_i\,R_i^\top v_i^\top\,P_{\beta_i} + 2(\beta_i^\top R_iv_i)\,P_{\beta_i}}{d_i^3} +\frac{P_{\beta_j}\,R_jv_j\,\beta_j^\top + \beta_j\,R_j^\top v_j^\top\,P_{\beta_j} + 2(\beta_j^\top R_jv_j)\,P_{\beta_j}}{d_j^3}
$$
以上をまとめると、スカラー化した確率関数
$$
{E}_{ij}^l = \exp\Biggl(-\frac{\sigma^2}{f^2}\,\operatorname{tr}\Bigl\{M^{-1}\Bigr\}\Biggr)
$$
（ただし $M = \frac{P_{\beta_i}}{d_i^2}+\frac{P_{\beta_j}}{d_j^2}$）の時間微分は、
$$
\dot{{E}}_{ij}^l = \frac{\sigma^2}{f^2}\,{E}_{ij}^l\,\operatorname{tr}\Bigl(M^{-1}\,\dot{M}\,M^{-1}\Bigr)
$$
であり、ここで $\dot{M}$ は上記の通り各エージェントの項の和として求まる。
### 制御入力に関する線形化

確率関数をCBFに適用するには，確率関数の時間微分を制御入力$v_i, v_j$についての線形な関数に変換する必要がある．ここで
$$
\begin{aligned}
\dot E_{ij}^l
&= -\frac{\sigma^2}{f^2}\,E_{ij}^l\,\dot T,\\
\dot T
&= \mathrm{tr}\left((M^{-1})^{\cdot}\right)
= -\mathrm{tr}\left(M^{-1}\dot M\,M^{-1}\right).
\end{aligned}
$$

ただし$M$は各エージェント $k\in\{i,j\}$ の速度を $w_k=R_k v_k$ とし
$$
\begin{aligned}
\dot M
&=\sum_{k\in\{i,j\}}
\frac{\mathrm d}{\mathrm dt}\left(\tfrac{P_{\beta_k}}{d_k^2}\right)\\
&=\sum_{k}
\frac{1}{d_k^2}\dot P_{\beta_k}
-\frac{2\dot d_k}{d_k^3}P_{\beta_k}\\
&=\sum_{k}
\frac{1}{d_k^3}\left[
P_{\beta_k}w_k\beta_k^\top
+\beta_k w_k^\top P_{\beta_k}
+2(\beta_k^\top w_k)P_{\beta_k}
\right].
\end{aligned}
$$
のように時間微分を計算することができる．

また，トレース演算は以下
$$
\begin{aligned}
&\mathrm{tr}\left(M^{-1}\dot M\,M^{-1}\right)\\
&=\sum_{k\in\{i,j\}}
\frac{1}{d_k^3}\left[
\underbrace{\mathrm{tr}(M^{-1}P_{\beta_k}w_k\beta_k^\top M^{-1})}_{t_{k,1}}
+\underbrace{\mathrm{tr}(M^{-1}\beta_k w_k^\top P_{\beta_k}M^{-1})}_{t_{k,2}}
+\underbrace{2(\beta_k^\top w_k)\mathrm{tr}(M^{-1}P_{\beta_k}M^{-1})}_{t_{k,3}}
\right].
\end{aligned}
$$
のように分解できる．ただし各項は
$\mathrm{tr}(A\,x\,y^\top\,B)=y^\top B A x$
により
$$
\begin{aligned}
t_{k,1}&=w_k^\top P_{\beta_k}M^{-2}\beta_k,\\
t_{k,2}&=w_k^\top P_{\beta_k}M^{-2}\beta_k,\\
t_{k,3}&=2(\beta_k^\top w_k)\,\chi_k,\quad
\chi_k=\mathrm{tr}(M^{-1}P_{\beta_k}M^{-1}).
\end{aligned}
$$
を得る。よって
$$
\begin{aligned}
\mathrm{tr}\left(M^{-1}\dot M\,M^{-1}\right)
&=\sum_{k}
\frac{2}{d_k^3}
w_k^\top\left(P_{\beta_k}M^{-2}\beta_k + \chi_k\beta_k\right).
\end{aligned}
$$

上記の変形により，
$w_k=R_k v_k$ を組み合わせ，
$$
\begin{aligned}
\dot E_{ij}^l
&=-\frac{\sigma^2}{f^2}E_{ij}^l\left(-\mathrm{tr}(M^{-1}\dot M\,M^{-1})\right)\\
&=\sum_{k\in\{i,j\}}
\lambda_k^\top v_k,
\end{aligned}
$$
$$
\lambda_k
:=-\frac{2\sigma^2}{f^2}\,
\frac{P_{ij}^l}{d_k^3}\,
R_k^\top\left(P_{\beta_k}M^{-2}\beta_k + \chi_k\beta_k\right).
$$
最終的に
$$
\begin{aligned}
\dot E_{ij}^l
= \lambda_i^\top v_i + \lambda_j^\top v_j
\end{aligned}
$$
が得られる。上述したFoV内に特徴点を保持するためのCBF制約に加えて，CRLBに関するCBF制約を追加するとQPは以下の様に修正できる．
$$
\begin{align}
 \min_{\xi_i, \xi_j}\:\sum_{i,j}(p^d_{i}-p_{i,{k+1}}-hR_{i,k}v_{i,k})^\top Q_1 & (p^d_{i}-p_{i,{k+1}}-hR_{i,k}v_{i,k})
+ 
\begin{bmatrix}
\omega_{i,k}\\v_{i,k}
\end{bmatrix}^\top Q_2
\begin{bmatrix}
\omega_{i,k}\\v_{i,k}
\end{bmatrix}\\
\mathrm{s.t.}
\sum_{l\in \mathcal{L}\subset\mathcal{C}_i \cap \mathcal{C}_j}(\prod_{k\neq l}(1-P_{ij}^k))P_j^l \langle \mathrm{grad}_\xi\:P_i^l,\xi_i\rangle&+\sum_{l\in \mathcal{L}\subset\mathcal{C}_i \cap \mathcal{C}_j}(\prod_{k\neq l}(1-P_{ij}^k))P_i^l \langle \mathrm{grad}_\xi\:P_j^l, \xi_j\rangle\\
&> -\gamma_0 (1-q_{fov}-\prod_{l\in\mathcal{L}}(1-P_{ij}^k))\\
\sum_{l\in \mathcal{L}\subset\mathcal{C}_i \cap \mathcal{C}_j}(\prod_{k\neq l}(1-E_{ij}^k))(\langle \mathrm{grad}_{p_i} E_{ij}^l, v_i\rangle&+\langle \mathrm{grad}_{p_j} E_{ij}^l, v_j\rangle)> -\gamma_1 (1-q_{est}-\prod_{l\in\mathcal{L}}(1-E_{ij}^k))
\end{align}
$$
ただし
$$
\begin{align}
\mathrm{grad}_{p_i} E_{ij}^l &= -\frac{2\sigma^2}{f^2}\,
\frac{E_{ij}^l}{d_{l}^3}\,
R_k^\top\left(P_{\beta_l}M^{-2}\beta_l + \mathrm{tr}(M^{-1}P_{\beta_k}M^{-1})\beta_l\right)\\
M &= \frac{P_{\beta_i}}{d_i^2} + \frac{P_{\beta_j}}{d_j^2}
\end{align}
$$
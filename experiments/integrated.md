
# 論文タイトル：SE(3)上における協調自己位置推定のための視野共有を保証する分散型CBF

メインテーゼ：CoVINSではsuperpoint特徴量などの画像特徴量のマッチングにより、自己位置推定に関する最適化問題のfactorを得ている。特徴量をマッチングするにはエージェントの視野錐台が重なっている必要がある。single agent問題に関してはstereo cameraの相対位置をconstかつgivenとして最適化問題に組み込み、multi agent問題に関しては視野錐台のoverrideは不可知であるために画像全体特徴量の類似度の一致などによってpassiveなevent trigger型としてアルゴリズムが構築されている。しかし、agent1とagent2が視野錐台を交差させ続けるための制御則(CBF等)に基づいてactive perceptionを行う場合、multi agentの自己位置推定においてinter-agentな特徴量のマッチング及び推定問題はエージェントをまたいだカメラ間の相対位置をgiven, もしくは最適化すべき双対変数として複数のエージェント位置を同時最適化できるはずである。さらに、active perceptionの枠組みで考えれば、CBFを用いた最適制御問題と自己位置推定問題も同一の目的関数の最小化問題として扱うことができるはずである。  
上記の仮定から、本セミナーではその手始めとして，代表的なCoVINSであるUAVを対象として、視野錐台を交差させ続けるための分散型CBF手法を提案する。

# 一次系システムにおける共有視野を保証するCBF

### 単一の特徴点を追従する安全制約

単一の特徴点を追従する安全制約については
元論文：Trimarchi, B., Schiano, F., & Tron, R. (2025). **A Control Barrier Function Candidate for Quadrotors with Limited Field of View**. _arXiv:2410.01277 [eess.SY]_
に大きく影響を受けている．

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

元論文：
R. Heusdens and G. Zhang, "Distributed Optimisation With Linear Equality and Inequality Constraints Using PDMM," in _IEEE Transactions on Signal and Information Processing over Networks_, vol. 10, pp. 294-306, 2024.

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
元論文：X. Tan and D. V. Dimarogonas, "Distributed Implementation of Control Barrier Functions for Multi-agent Systems," in _IEEE Control Systems Letters_, vol. 6, pp. 1879-1884, 2022.

https://people.kth.se/~dimos/pdfs/DistributedCBF_LCSS_2022.pdf#:~:text=Abstract%E2%80%94%20In%20this%20work%2C%20we,Instead%2C%20to%20solve%20the%20quadratic

最適化中に元の制約式を満たすことを保証する？ADMMベースのCBF分散化手法

# 二次系システムにおける共有視野を保証するHOCBF

共有視野の分散CBFを実機に適用するため，ドローンのダイナミクスを考慮したCBF付きQPの定式化を試みる．
## SE(3)における離散ダイナミクス

SE(3)における離散ダイナミクスは
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
&= Mv_k+h(Mv_k\times\Omega_k-MgR^\top e_z+f)\\
\end{align}
$$
ここで回転運動について，後述する非ホロノミック系に対応するため以下のような近似を行う．
$$
\begin{align}
R_{k+1}&=R_kF_k\simeq R_kF_{k+1}\\
p_{k+1}&=p_k+hR_{k}v_{k}\simeq p_k+hR_{k+1}v_{k+1}
\end{align}
$$
の仮定を認めると
姿勢更新について
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
位置更新について
$$
\begin{align}
p_{k+1}&=p_k+hR_{k+1}v_{k+1}\\
&=p_k+h(R_k+hR_k\Omega_{k+1}^\times)(v_k+h(v_k\times \Omega_{k}-gR_k^\top e_c+R_kM^{-1}f))\\
&=p_k+h(R_k+hR_k\Omega^\times_k+h^2[J^{-1}\tau_k]_\times)(v_k+h(v_k\times \Omega_{k}-gR_k^\top e_c+R_kM^{-1}f))\\
&=p_k+hR_kv_k+h^2(-ge_z+R_kM^{-1}f)+h^3[J^{-1}\tau_k]_\times v_k+\mathcal{O}(h^4)
\end{align}
$$
となり逐次ステップでトルク$\tau_k$によって状態$(p_{k+1},R_{k+1})\in \mathrm{SE}(3)$を操作可能になる．

## ホロノミック系
まず，$f=(f_x, f_y, f_z)\in \mathbb{R}^3$(ホロノミック系)におけるHOCBF-QPを考える．
誤差を
$$
\begin{align}
p^d_{k}-p_{k+1}&=\underbrace{p^d_{k}-p_k-hR_kv_k}_{e_k}-h^2(-ge_z+R_kM^{-1}f)-h^3[J^{-1}\tau_k]_\times v_k\\
&=\underbrace{e_k+h^2ge_z}_{\tilde e_k}-h^2M^{-1}R_kf+h^3v_k^\times J^{-1}\tau_k\\
&=\tilde e_k-h^2M^{-1}R_kf+h^3v_k^\times J^{-1}\tau_k\\
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
のように定義する．
最小化したい目的関数は
$$
\begin{align}
J&=\frac{1}{2}\|\tilde e_k+A\|^2+\frac{1}{2}\begin{bmatrix}
A_g+M^{-1}f\\J^{-1}\tau
\end{bmatrix}^\top B \begin{bmatrix}
A_g+M^{-1}f\\J^{-1}\tau
\end{bmatrix}\\
&\propto \frac{1}{2}u_k^\top A^\top Au_k +(A^\top \tilde e_k)^\top u_k\\
&\quad+\frac{1}{2}u_k^\top 
\underbrace{\begin{bmatrix}
M^{-2}B_1&0\\0&(J^{-1})^\top B_2J^{-1}
\end{bmatrix}}_{B'_1\in\mathbb{R}^{6\times 6}} u_k+\underbrace{\begin{bmatrix}
M^{-1}B_1 A_g^\top  e_z\\0
\end{bmatrix}^\top}_{B'_2\in\mathbb{R}^{6}}u_k\\
&\propto \frac{1}{2}u_k^\top(A^\top A+B'_1)u_k+(A^\top \tilde e_k+B'_2)^\top u_k\\
A_g&=v^\times\Omega-gR^\top e_z
\end{align}
$$

よって解くべきQPは
$$
\begin{align}
\min_{u_k}&\: \frac{1}{2}u_k^\top(A^\top A+B'_1)u_k+(A^\top \tilde e_k+B'_2)^\top u_k\\
u_k &= \begin{bmatrix} f_k \\ \tau_k \end{bmatrix} \in \mathbb{R}^6\\
A&=\begin{bmatrix}
-h^2M^{-1}R_k&h^3v_k \times J^{-1}
\end{bmatrix}\in\mathbb{R}^{3\times 6}\\
B'_1&=\begin{bmatrix}
M^{-2}B_1&0\\0&(J^{-1})^\top B_2J^{-1}
\end{bmatrix}\in\mathbb{R}^{6\times 6}\\
B'_2&=\begin{bmatrix}
M^{-1}B_1 A_g^\top  e_z\\0
\end{bmatrix}\in\mathbb{R}^{6}\\
\tilde e_k&=p^d_{k}-p_k-hR_kv_k+h^2ge_z\\
\end{align}
$$
## 単一の特徴点を追従するHOCBF制約

$$
h=\beta_l^{\top}(p_i)R_ie_c-\cos\Psi_\mathcal{F}
$$
とすると，安全集合の２階微分は
$$
\begin{align}
\ddot h &= \underbrace{-\frac{e_c^\top R^\top P_\beta R}{d}\dot v}_{\langle \mathrm{grad}_ph,\dot v\rangle}
+\underbrace{\frac{d}{dt}\left(-\frac{e_c^\top R^\top P_\beta R}{d}\right)v}_{\langle \mathrm{Hess}_ph[v],v\rangle
+\langle \mathrm{Hess}_ph[v],\Omega\rangle}\\
&\qquad 
+\underbrace{\left(\frac{P_\beta}{d}Rv\right)^\top R e_c^\times\Omega}_{\langle \mathrm{Hess}_Rh[\Omega],v\rangle}
\underbrace{-\beta^\top R\Omega^\times e_c^\times \Omega}_{\langle \mathrm{Hess}_Rh[\Omega],\Omega\rangle}
\underbrace{-\beta^\top Re_c^\times \dot\Omega}_{\langle \mathrm{grad}_Rh,\dot \Omega\rangle}\\
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
\langle \mathrm{Hess}_ph[v],\Omega\rangle&=\langle \mathrm{Hess}_Rh[\Omega],v\rangle=v^\top R^\top \frac{P_\beta R e_c^\times}{d}\Omega,\\
\langle \mathrm{Hess}_Rh[\Omega],\Omega\rangle&=\omega^\top[R^\top\beta]_\times e_c^\times\Omega,\\
\mathrm{grad}_ph&=-\frac{e_c^\top R^\top P_\beta R}{d}\\
\mathrm{grad}_Rh&=-\beta^\top Re_c^\times\\
\end{align}
$$

$$
\begin{align}
\frac{d}{dt}\left(-\frac{e_c^\top R^\top P_\beta R}{d}\right)v
&=\underbrace{v^\top R^\top \frac{P_\beta R e_c^\times}{d}\Omega}_{\langle \mathrm{Hess}_ph[v],\Omega\rangle}
-\frac{z^\top\dot P_\beta}{d}Rv
-\frac{z^\top P_\beta}{d}R\Omega^\times v
-v^\top R^\top\frac{P_\beta z\beta^\top}{d^2}Rv\\
&=\langle \mathrm{Hess}_ph[v],\Omega\rangle\\
&\quad \underbrace{-v^\top R^\top\frac{\beta(z^\top P_\beta)+(z^\top \beta)P_\beta+P_\beta z\beta^\top}{d^2}Rv-\frac{z^\top P_\beta}{d}R\Omega^\times v}_{\langle \mathrm{Hess}_ph[v],v\rangle}\\
&=\langle \mathrm{Hess}_ph[v],v\rangle
+\langle \mathrm{Hess}_ph[v],\Omega\rangle\\
\end{align}
$$
よって２次系におけるHOCBFは
$$
\begin{align}
&\underbrace{-\frac{e_c^\top R^\top P_\beta R}{d}\dot v}_{\langle \mathrm{grad}_ph,\dot v\rangle}
+\underbrace{v^\top R^\top \frac{P_\beta R e_c^\times}{d}\Omega}_{\langle \mathrm{Hess}_ph[v],\Omega\rangle}\\
&\underbrace{-v^\top R^\top\frac{\beta(z^\top P_\beta)+(z^\top \beta)P_\beta+P_\beta(Re_c)\beta^\top}{d^2}Rv-\frac{(Re_c)^\top P_\beta}{d}R\Omega^\times v}_{\langle \mathrm{Hess}_ph[v],v\rangle}\\
&\underbrace{-\beta^\top Re_c^\times \dot\Omega}_{\langle \mathrm{grad}_Rh,\dot \Omega\rangle}
\underbrace{-\beta^\top R\Omega^\times e_c^\times \Omega}_{\langle \mathrm{Hess}_Rh[\Omega],\Omega\rangle}
+\underbrace{\left(\frac{P_\beta}{d}Rv\right)^\top R e_c^\times\Omega}_{\langle \mathrm{Hess}_Rh[\Omega],v\rangle}\\
&+(\gamma_0+\gamma_1)(\underbrace{-\beta^\top Re_c^\times\Omega}_{\langle \mathrm{grad}_Rh,\Omega\rangle}\underbrace{-\frac{e_c^\top R^\top P_\beta R}{d}v}_{\langle \mathrm{grad}_ph,v\rangle})+\gamma_0\gamma_1(\beta_l^{\top}Re_c-\cos\Psi_\mathcal{F})\geq0
\end{align}
$$
$\dot v, \dot \Omega$を書き下すと
$$
\dot \Omega\simeq J^{-1}\tau, \quad\dot v \simeq v^\times\Omega-gR^\top e_z+M^{-1}f
$$
として
$$
\begin{align}
&\underbrace{-\beta^\top Re_c^\times J^{-1}\tau}_{\langle \mathrm{grad}_Rh,\dot \Omega\rangle}
\underbrace{-\frac{e_c^\top R^\top P_\beta R}{d}(v^\times\Omega-gR^\top e_z+M^{-1}f)}_{\langle \mathrm{grad}_ph,\dot v\rangle}\\
&+\underbrace{v^\top R^\top \frac{P_\beta R e_c^\times}{d}\Omega}_{\langle \mathrm{Hess}_ph[v],\Omega\rangle}\\
&\underbrace{-v^\top R^\top\frac{\beta(z^\top P_\beta)+(z^\top \beta)P_\beta+P_\beta(Re_c)\beta^\top}{d^2}Rv-\frac{(Re_c)^\top P_\beta}{d}R\Omega^\times v}_{\langle \mathrm{Hess}_ph[v],v\rangle}\\
&
\underbrace{-\beta^\top R\Omega^\times e_c^\times \Omega}_{\langle \mathrm{Hess}_Rh[\Omega],\Omega\rangle}
+\underbrace{\left(\frac{P_\beta}{d}Rv\right)^\top R e_c^\times\Omega}_{\langle \mathrm{Hess}_Rh[\Omega],v\rangle}\\
&+2\gamma_0(\underbrace{-\beta^\top Re_c^\times\Omega}_{\langle \mathrm{grad}_Rh,\Omega\rangle}\underbrace{-\frac{e_c^\top R^\top P_\beta R}{d}v}_{\langle \mathrm{grad}_ph,v\rangle})+\gamma_0^2(\beta_l^{\top}Re_c-\cos\Psi_\mathcal{F})\geq0
\end{align}
$$
となる．
したがって、HOCBF制約付きQPは以下のように定式化される：
$$
\begin{align}
\min_{u_k}&\: \frac{1}{2}u_k^\top(A^\top A+B'_1)u_k+(A^\top \tilde e_k+B'_2)^\top u_k\\
\mathrm{s.t.}&\: Cu_k \leq b
\end{align}
$$
$$
\begin{align}
C &= \begin{bmatrix} \frac{e_c^\top R^\top P_\beta R}{d} M^{-1} & \beta^\top Re_c^\times J^{-1} \end{bmatrix} \in \mathbb{R}^{1\times 6}\\
b &= \langle \mathrm{Hess}_ph[v],\Omega\rangle + \langle \mathrm{Hess}_ph[v],v\rangle + \langle \mathrm{Hess}_Rh[\Omega],v\rangle + \langle \mathrm{Hess}_Rh[\Omega],\Omega\rangle\\
&\quad + 2\gamma_0(\langle \mathrm{grad}_Rh,\Omega\rangle + \langle \mathrm{grad}_ph,v\rangle) + \gamma_0^2(\beta_l^{\top}Re_c-\cos\Psi_\mathcal{F})\\
&\quad - \frac{e_c^\top R^\top P_\beta R}{d}(v^\times\Omega-gR^\top e_z)
\end{align}
$$
となる．

## 非ホロノミック系

ただしドローンの非ホロノミック系力学モデルに合わせるため
$$
\begin{align}
f_k&\mapsto fe_z, f_k\in \mathbb{R}\\
u_k&=\begin{bmatrix}
f_k\\
\tau_k
\end{bmatrix}\in\mathbb{R}^4
\end{align}
$$
とする．非ホロノミック系におけるHOCBF制約付きQPは以下のように定式化される：
$$
\begin{align}
\min_{u_k}&\: \frac{1}{2}u_k^\top(A^\top A+B'_1)u_k+(A^\top \tilde e_k+B'_2)^\top u_k\\
\mathrm{s.t.}&\: Cu_k \leq b
\end{align}
$$
ただし
$$
\begin{align}
u_k &= \begin{bmatrix} f_k \\ \tau_k \end{bmatrix} \in \mathbb{R}^4\\
A &= \begin{bmatrix} -h^2M^{-1}R_ke_c & h^3v_k \times J^{-1} \end{bmatrix} \in \mathbb{R}^{3\times 4}\\
B'_1&=\begin{bmatrix}
M^{-2}b_1&0\\0&(J^{-1})^\top B_2J^{-1}
\end{bmatrix}\in\mathbb{R}^{4\times 6}\\
B'_2&=\begin{bmatrix}
M^{-1}b_1 A_g^\top  e_z\\0
\end{bmatrix}\in\mathbb{R}^{4}\\
\tilde e_k &= p^d_{k}-p_k-hR_kv_k+h^2ge_c\\
C &= \begin{bmatrix} \frac{e_c^\top R^\top P_\beta R}{d}e_z M^{-1} & \beta^\top Re_c^\times J^{-1} \end{bmatrix} \in \mathbb{R}^{1\times 4}\\
b &= \langle \mathrm{Hess}_ph[v],\Omega\rangle + \langle \mathrm{Hess}_ph[v],v\rangle + \langle \mathrm{Hess}_Rh[\Omega],v\rangle + \langle \mathrm{Hess}_Rh[\Omega],\Omega\rangle\\
&\quad + 2\gamma_0(\langle \mathrm{grad}_Rh,\Omega\rangle + \langle \mathrm{grad}_ph,v\rangle) + \gamma_0^2(\beta_l^{\top}Re_c-\cos\Psi_\mathcal{F})\\
&\quad - \frac{e_c^\top R^\top P_\beta R}{d}(v^\times\Omega-gR^\top e_z)
\end{align}
$$
である．

## 複数の特徴点を追従する安全制約
以下非ホロノミック系についてHOCBFの設計を行う．
一次系の場合と同様に安全集合を
$$
\begin{align}
B_{i}&=1-q-\eta_{i}\\
\eta_{i}&=\prod_{l\in\mathcal{L}}(1-\phi_{i}^l)
\end{align}
$$
とする．
安全集合の一階微分は一次系の場合と同様に
$$
\begin{align}
\dot B_{i}&=-\dot\eta_{i}\\
&=-\frac{d}{dt}\prod_{l\in\mathcal{L}}(1-\phi_{i}^l)\\
&=\sum_{l\in \mathcal{L}}(\prod_{k\neq l}(1-\phi_{i}^k))\dot \phi^l_{i}\\
\dot \phi^l_{i} &= \left\{ \begin{array}{ll}
\dot P_i^l & \mathrm{if}  \quad  q_l\in\mathcal{C}_i\\
0 & \mathrm{if}  \quad  q_l\in \mathcal{L}\setminus\mathcal{C}_i
\end{array} \right.
\end{align}
$$
$$
\begin{align}
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
安全集合の二階微分は
$$
\begin{align}
\ddot B_i &= \frac{d}{dt}\dot B_i\\
&= \frac{d}{dt}\left(\sum_{l\in \mathcal{L}}(\prod_{k\neq l}(1-\phi_{i}^k))\dot \phi^l_{i}\right)\\
&= \sum_{l\in \mathcal{L}}\frac{d}{dt}\left((\prod_{k\neq l}(1-\phi_{i}^k))\dot \phi^l_{i}\right)\\
&= \sum_{l\in \mathcal{L}}\left(\frac{d}{dt}(\prod_{k\neq l}(1-\phi_{i}^k))\dot \phi^l_{i} + (\prod_{k\neq l}(1-\phi_{i}^k))\ddot \phi^l_{i}\right)\\
&= \sum_{l\in \mathcal{L}}\left(-\sum_{j\neq l}(\prod_{m\neq j,l}(1-\phi_{i}^m))\dot\phi_i^j\dot \phi^l_{i} + (\prod_{k\neq l}(1-\phi_{i}^k))\ddot \phi^l_{i}\right)
\end{align}
$$
ここで、$\ddot \phi^l_{i}$は$P_i^l$の二階微分であり、以下のように計算できる：
$$
\begin{align}
\ddot \phi^l_{i} &= \left\{ \begin{array}{ll}
\ddot P_i^l & \mathrm{if}  \quad  q_l\in\mathcal{C}_i\\
0 & \mathrm{if}  \quad  q_l\in \mathcal{L}\setminus\mathcal{C}_i
\end{array} \right.\\
\ddot P_i^l &= \frac{d}{dt}\dot P_i^l\\
&= \frac{d}{dt}\langle \mathrm{grad}\:P_i^l, \xi_W\rangle\\
&= \langle \mathrm{Hess}\:P_i^l[\xi_W], \xi_W\rangle + \langle \mathrm{grad}\:P_i^l, \dot\xi_W\rangle
\end{align}
$$

ここで、$\mathrm{Hess}\:P_i^l$は$P_i^l$のヘッシアン行列であり、$\dot\xi_W$は制御入力に依存する項である。$\mathrm{Hess}\:P_i^l$は以下のように分解できる：
$$
\begin{align}
\langle \mathrm{Hess}\:P_i^l[\xi_W], \xi_W\rangle &= \langle \mathrm{Hess}_p\:P_i^l[v], v\rangle + \langle \mathrm{Hess}_p\:P_i^l[v], \omega\rangle + \langle \mathrm{Hess}_R\:P_i^l[\omega], v\rangle + \langle \mathrm{Hess}_R\:P_i^l[\omega], \omega\rangle
\end{align}
$$

また、$\langle \mathrm{grad}\:P_i^l, \dot\xi_W\rangle$は以下のように分解できる：
$$
\begin{align}
\langle \mathrm{grad}\:P_i^l, \dot\xi_W\rangle &= \langle \mathrm{grad}_p\:P_i^l, \dot v\rangle + \langle \mathrm{grad}_R\:P_i^l, \dot\omega\rangle
\end{align}
$$

これらを用いて、HOCBFの制約は以下のように表される：
$$
\begin{align}
\ddot B_i + \gamma_1\dot B_i + \gamma_0 B_i \geq 0
\end{align}
$$

ここで、$\gamma_0, \gamma_1$は正の定数である。この制約を展開すると：
$$
\begin{align}
&\sum_{l\in \mathcal{L}}\left(-\sum_{j\neq l}(\prod_{m\neq j,l}(1-\phi_{i}^m))\dot\phi_i^j\dot \phi^l_{i} + (\prod_{k\neq l}(1-\phi_{i}^k))\ddot \phi^l_{i}\right)\\
&+ \gamma_1\sum_{l\in \mathcal{L}}(\prod_{k\neq l}(1-\phi_{i}^k))\dot \phi^l_{i}\\
&+ \gamma_0(1-q-\prod_{l\in\mathcal{L}}(1-\phi_{i}^l)) \geq 0
\end{align}
$$

制御入力$u_k = [f_k, \tau_k]^\top$に依存する項は$\ddot \phi^l_{i}$の中の$\langle \mathrm{grad}_p\:P_i^l, \dot v\rangle$と$\langle \mathrm{grad}_R\:P_i^l, \dot\omega\rangle$である。これらを制御入力について整理すると：
$$
\begin{align}
\langle \mathrm{grad}_p\:P_i^l, \dot v\rangle &= \frac{1}{1-\cos \psi_\mathcal{F}}(-\frac{e_c^\top R_i^\top P_{\beta_l}}{d})\dot v\\
\langle \mathrm{grad}_R\:P_i^l, \dot\omega\rangle &= \frac{1}{1-\cos \psi_\mathcal{F}}(-\beta_l^\top(p_i) R_i [e_c]_\times)\dot\omega
\end{align}
$$

$\dot v$と$\dot \omega$を制御入力$u_k = [f_k, \tau_k]^\top$で表すと：
$$
\begin{align}
\dot v &= Mv^\times\Omega-mgR^\top e_z+f\\
\dot\omega &= J^{-1}\tau
\end{align}
$$

これらを代入して整理すると、以下のような制約付きQPが得られる：
$$
\begin{align}
\min_{u_k}&\: \frac{1}{2}u_k^\top(A^\top A+B)u_k+(A^\top \tilde e_k)^\top u_k\\
\mathrm{s.t.}&\: C_{\mathrm{multi}}u_k \geq b_{\mathrm{multi}}
\end{align}
$$

ここで、
$$
\begin{align}
C_{\mathrm{multi}} &= \begin{bmatrix}
\sum_{l\in \mathcal{L}\subset\mathcal{C}_i}(\prod_{k\neq l}(1-\phi_{i}^k)) \frac{e_c^\top R_i^\top P_{\beta_l}}{(1-\cos \psi_\mathcal{F})d} & \sum_{l\in \mathcal{L}\subset\mathcal{C}_i}(\prod_{k\neq l}(1-\phi_{i}^k)) \frac{\beta_l^\top(p_i) R_i [e_c]_\times}{1-\cos \psi_\mathcal{F}}J^{-1}
\end{bmatrix}\\
b_{\mathrm{multi}} &= -\sum_{l\in \mathcal{L}}\left(-\sum_{j\neq l}(\prod_{m\neq j,l}(1-\phi_{i}^m))\dot\phi_i^j\dot \phi^l_{i} + (\prod_{k\neq l}(1-\phi_{i}^k))(\langle \mathrm{Hess}\:P_i^l[\xi_W], \xi_W\rangle)\right)\\
&\quad - \gamma_1\sum_{l\in \mathcal{L}}(\prod_{k\neq l}(1-\phi_{i}^k))\dot \phi^l_{i}\\
&\quad - \gamma_0(1-q-\prod_{l\in\mathcal{L}}(1-\phi_{i}^l))\\
&\quad + \sum_{l\in \mathcal{L}\subset\mathcal{C}_i}(\prod_{k\neq l}(1-\phi_{i}^k)) \frac{e_c^\top R_i^\top P_{\beta_l}}{(1-\cos \psi_\mathcal{F})d}(Mv^\times\Omega-mgR^\top e_z)
\end{align}
$$

### 複数のエージェントが共通の特徴点を追従する安全制約付きQP（HOCBF）

複数のエージェントが共通の特徴点を追従する場合、安全集合を以下のように定義する：
$$
\begin{align}
B_{ij}&=1-q-\eta_{ij}\\
\eta_{ij}&=\prod_{l\in\mathcal{L}}(1-\phi_{ij}^l)
\end{align}
$$

ここで、$\phi_{ij}^l$は特徴点$q_l\in \mathcal{L}$によってエッジ$(i,j)\in \mathcal{E}$における推定が成り立っている確率であり、以下のように定義される：
$$
\begin{align}
\phi_{ij}^l &= \left\{ \begin{array}{ll}
P_i^lP_j^l & \mathrm{if}  \quad  q_l\in\mathcal{C}_i \cap \mathcal{C}_j\\
0 & \mathrm{if}  \quad  q_l\in \mathcal{L}\setminus\mathcal{C}_i \cap \mathcal{C}_j
\end{array} \right.\\
\mathrm{where} \quad P_i^l &= \frac{\beta_l^\top(p_i) R_i e_c -\cos\Psi_\mathcal{F} }{1-\cos\Psi_\mathcal{F}}
\end{align}
$$

安全集合$B_{ij}$の時間微分は以下のように計算される：
$$
\begin{align}
\dot B_{ij}&=-\dot\eta_{ij}\\
&=-\frac{d}{dt}\prod_{l\in\mathcal{L}}(1-\phi_{ij}^l)\\
&=\sum_{l\in \mathcal{L}}(\prod_{k\neq l}(1-\phi_{ij}^k))\dot \phi^l_{ij}\\
\dot \phi^l_{ij} &= \left\{ \begin{array}{ll}
\dot P_i^lP_j^l+P_i^l\dot P_j^l & \mathrm{if}  \quad  q_l\in\mathcal{C}_i \cap \mathcal{C}_j\\
0 & \mathrm{if}  \quad  q_l\in \mathcal{L}\setminus\mathcal{C}_i \cap \mathcal{C}_j
\end{array} \right.
\end{align}
$$

エージェントごとの制御入力について分解すると：
$$
\begin{align}
\dot B_{ij}&=-\dot\eta_{ij}\\
&=-\frac{d}{dt}\prod_{l\in\mathcal{L}}(1-\phi_{ij}^l)\\
&=\sum_{l\in \mathcal{L}}(\prod_{k\neq l}(1-\phi_{ij}^k))\dot \phi^l_{ij}\\
&= \sum_{l\in \mathcal{L}\subset\mathcal{C}_i \cap \mathcal{C}_j}(\prod_{k\neq l}(1-\phi_{ij}^k))P_j^l\dot P_i^l+\sum_{l\in \mathcal{L}\subset\mathcal{C}_i \cap \mathcal{C}_j}(\prod_{k\neq l}(1-\phi_{ij}^k))P_i^l\dot P_j^l
\end{align}
$$

安全集合$B_{ij}$の二階微分を計算するために、$\dot B_{ij}$の時間微分を計算する：
$$
\begin{align}
\ddot B_{ij} &= \frac{d}{dt}\dot B_{ij}\\
&= \frac{d}{dt}\left(\sum_{l\in \mathcal{L}}(\prod_{k\neq l}(1-\phi_{ij}^k))\dot \phi^l_{ij}\right)\\
&= \sum_{l\in \mathcal{L}}\frac{d}{dt}\left((\prod_{k\neq l}(1-\phi_{ij}^k))\dot \phi^l_{ij}\right)\\
&= \sum_{l\in \mathcal{L}}\left(\frac{d}{dt}(\prod_{k\neq l}(1-\phi_{ij}^k))\dot \phi^l_{ij} + (\prod_{k\neq l}(1-\phi_{ij}^k))\ddot \phi^l_{ij}\right)\\
&= \sum_{l\in \mathcal{L}}\left(-\sum_{j\neq l}(\prod_{m\neq j,l}(1-\phi_{ij}^m))\dot\phi_{ij}^j\dot \phi^l_{ij} + (\prod_{k\neq l}(1-\phi_{ij}^k))\ddot \phi^l_{ij}\right)
\end{align}
$$

ここで、$\ddot \phi^l_{ij}$は$\phi_{ij}^l$の二階微分であり、以下のように計算できる：
$$
\begin{align}
\ddot \phi^l_{ij} &= \left\{ \begin{array}{ll}
\ddot P_i^lP_j^l + 2\dot P_i^l\dot P_j^l + P_i^l\ddot P_j^l & \mathrm{if}  \quad  q_l\in\mathcal{C}_i \cap \mathcal{C}_j\\
0 & \mathrm{if}  \quad  q_l\in \mathcal{L}\setminus\mathcal{C}_i \cap \mathcal{C}_j
\end{array} \right.
\end{align}
$$

$\ddot P_i^l$と$\ddot P_j^l$は$P_i^l$と$P_j^l$の二階微分であり、以下のように計算できる：
$$
\begin{align}
\ddot P_i^l &= \frac{d}{dt}\dot P_i^l\\
&= \frac{d}{dt}\langle \mathrm{grad}\:P_i^l, \xi_{W,i}\rangle\\
&= \langle \mathrm{Hess}\:P_i^l[\xi_{W,i}], \xi_{W,i}\rangle + \langle \mathrm{grad}\:P_i^l, \dot\xi_{W,i}\rangle\\
\ddot P_j^l &= \frac{d}{dt}\dot P_j^l\\
&= \frac{d}{dt}\langle \mathrm{grad}\:P_j^l, \xi_{W,j}\rangle\\
&= \langle \mathrm{Hess}\:P_j^l[\xi_{W,j}], \xi_{W,j}\rangle + \langle \mathrm{grad}\:P_j^l, \dot\xi_{W,j}\rangle
\end{align}
$$

ここで、$\mathrm{Hess}\:P_i^l$と$\mathrm{Hess}\:P_j^l$は$P_i^l$と$P_j^l$のヘッシアン行列であり、$\dot\xi_{W,i}$と$\dot\xi_{W,j}$は制御入力に依存する項である。$\mathrm{Hess}\:P_i^l$と$\mathrm{Hess}\:P_j^l$は以下のように分解できる：
$$
\begin{align}
\langle \mathrm{Hess}\:P_i^l[\xi_{W,i}], \xi_{W,i}\rangle &= \langle \mathrm{Hess}_p\:P_i^l[v_i], v_i\rangle + \langle \mathrm{Hess}_p\:P_i^l[v_i], \omega_i\rangle + \langle \mathrm{Hess}_R\:P_i^l[\omega_i], v_i\rangle + \langle \mathrm{Hess}_R\:P_i^l[\omega_i], \omega_i\rangle\\
\langle \mathrm{Hess}\:P_j^l[\xi_{W,j}], \xi_{W,j}\rangle &= \langle \mathrm{Hess}_p\:P_j^l[v_j], v_j\rangle + \langle \mathrm{Hess}_p\:P_j^l[v_j], \omega_j\rangle + \langle \mathrm{Hess}_R\:P_j^l[\omega_j], v_j\rangle + \langle \mathrm{Hess}_R\:P_j^l[\omega_j], \omega_j\rangle
\end{align}
$$

また、$\langle \mathrm{grad}\:P_i^l, \dot\xi_{W,i}\rangle$と$\langle \mathrm{grad}\:P_j^l, \dot\xi_{W,j}\rangle$は以下のように分解できる：
$$
\begin{align}
\langle \mathrm{grad}\:P_i^l, \dot\xi_{W,i}\rangle &= \langle \mathrm{grad}_p\:P_i^l, \dot v_i\rangle + \langle \mathrm{grad}_R\:P_i^l, \dot\omega_i\rangle\\
\langle \mathrm{grad}\:P_j^l, \dot\xi_{W,j}\rangle &= \langle \mathrm{grad}_p\:P_j^l, \dot v_j\rangle + \langle \mathrm{grad}_R\:P_j^l, \dot\omega_j\rangle
\end{align}
$$

これらを用いて、HOCBFの制約は以下のように表される：
$$
\begin{align}
\ddot B_{ij} + \gamma_1\dot B_{ij} + \gamma_0 B_{ij} \geq 0
\end{align}
$$

ここで、$\gamma_0, \gamma_1$は正の定数である。この制約を展開すると：
$$
\begin{align}
&\sum_{l\in \mathcal{L}}\left(-\sum_{j\neq l}(\prod_{m\neq j,l}(1-\phi_{ij}^m))\dot\phi_{ij}^j\dot \phi^l_{ij} + (\prod_{k\neq l}(1-\phi_{ij}^k))\ddot \phi^l_{ij}\right)\\
&+ \gamma_1\sum_{l\in \mathcal{L}}(\prod_{k\neq l}(1-\phi_{ij}^k))\dot \phi^l_{ij}\\
&+ \gamma_0(1-q-\prod_{l\in\mathcal{L}}(1-\phi_{ij}^l)) \geq 0
\end{align}
$$

制御入力$u_i = [f_i, \tau_i]^\top$と$u_j = [f_j, \tau_j]^\top$に依存する項は$\ddot \phi^l_{ij}$の中の$\langle \mathrm{grad}_p\:P_i^l, \dot v_i\rangle$、$\langle \mathrm{grad}_R\:P_i^l, \dot\omega_i\rangle$、$\langle \mathrm{grad}_p\:P_j^l, \dot v_j\rangle$、$\langle \mathrm{grad}_R\:P_j^l, \dot\omega_j\rangle$である。これらを制御入力について整理すると：
$$
\begin{align}
\langle \mathrm{grad}_p\:P_i^l, \dot v_i\rangle &= \frac{1}{1-\cos \psi_\mathcal{F}}(-\frac{e_c^\top R_i^\top P_{\beta_l}}{d})\dot v_i\\
\langle \mathrm{grad}_R\:P_i^l, \dot\omega_i\rangle &= \frac{1}{1-\cos \psi_\mathcal{F}}(-\beta_l^\top(p_i) R_i [e_c]_\times)\dot\omega_i\\
\langle \mathrm{grad}_p\:P_j^l, \dot v_j\rangle &= \frac{1}{1-\cos \psi_\mathcal{F}}(-\frac{e_c^\top R_j^\top P_{\beta_l}}{d})\dot v_j\\
\langle \mathrm{grad}_R\:P_j^l, \dot\omega_j\rangle &= \frac{1}{1-\cos \psi_\mathcal{F}}(-\beta_l^\top(p_j) R_j [e_c]_\times)\dot\omega_j
\end{align}
$$

$\dot v_i$、$\dot \omega_i$、$\dot v_j$、$\dot \omega_j$を制御入力$u_i = [f_i, \tau_i]^\top$と$u_j = [f_j, \tau_j]^\top$で表すと：
$$
\begin{align}
\dot v_i &= Mv_i^\times\Omega_i-mgR_i^\top e_z+f_i\\
\dot\omega_i &= J^{-1}\tau_i\\
\dot v_j &= Mv_j^\times\Omega_j-mgR_j^\top e_z+f_j\\
\dot\omega_j &= J^{-1}\tau_j
\end{align}
$$

これらを代入して整理すると、以下のような制約付きQPが得られる：
$$
\begin{align}
\min_{u_i, u_j}&\: \frac{1}{2}u_i^\top(A_i^\top A_i+B_i)u_i+(A_i^\top \tilde e_i)^\top u_i + \frac{1}{2}u_j^\top(A_j^\top A_j+B_j)u_j+(A_j^\top \tilde e_j)^\top u_j\\
\mathrm{s.t.}&\: C_i u_i + C_j u_j \geq b_{ij}
\end{align}
$$

ここで、
$$
\begin{align}
C_i &= \begin{bmatrix}
\sum_{l\in \mathcal{L}\subset\mathcal{C}_i \cap \mathcal{C}_j}(\prod_{k\neq l}(1-\phi_{ij}^k))P_j^l\frac{e_c^\top R_i^\top P_{\beta_l}}{(1-\cos \psi_\mathcal{F})d} & \sum_{l\in \mathcal{L}\subset\mathcal{C}_i \cap \mathcal{C}_j}(\prod_{k\neq l}(1-\phi_{ij}^k))P_j^l \frac{\beta_l^\top(p_i) R_i [e_c]_\times}{1-\cos \psi_\mathcal{F}}J^{-1}
\end{bmatrix}\\
C_j &= \begin{bmatrix}
\sum_{l\in \mathcal{L}\subset\mathcal{C}_i \cap \mathcal{C}_j}(\prod_{k\neq l}(1-\phi_{ij}^k))P_i^l\frac{e_c^\top R_j^\top P_{\beta_l}}{(1-\cos \psi_\mathcal{F})d} & \sum_{l\in \mathcal{L}\subset\mathcal{C}_i \cap \mathcal{C}_j}(\prod_{k\neq l}(1-\phi_{ij}^k))P_i^l \frac{\beta_l^\top(p_j) R_j [e_c]_\times}{1-\cos \psi_\mathcal{F}}J^{-1}
\end{bmatrix}\\
b_{ij} &= -\sum_{l\in \mathcal{L}}\left(-\sum_{j\neq l}(\prod_{m\neq j,l}(1-\phi_{ij}^m))\dot\phi_{ij}^j\dot \phi^l_{ij} + (\prod_{k\neq l}(1-\phi_{ij}^k))(\langle \mathrm{Hess}\:P_i^l[\xi_{W,i}], \xi_{W,i}\rangle P_j^l + 2\dot P_i^l\dot P_j^l + P_i^l\langle \mathrm{Hess}\:P_j^l[\xi_{W,j}], \xi_{W,j}\rangle)\right)\\
&\quad - \gamma_1\sum_{l\in \mathcal{L}}(\prod_{k\neq l}(1-\phi_{ij}^k))\dot \phi^l_{ij}\\
&\quad - \gamma_0(1-q-\prod_{l\in\mathcal{L}}(1-\phi_{ij}^l))\\
&\quad + \sum_{l\in \mathcal{L}\subset\mathcal{C}_i \cap \mathcal{C}_j}(\prod_{k\neq l}(1-\phi_{ij}^k))P_j^l \frac{e_c^\top R_i^\top P_{\beta_l}}{(1-\cos \psi_\mathcal{F})d}(Mv_i^\times\Omega_i-mgR_i^\top e_z)\\
&\quad + \sum_{l\in \mathcal{L}\subset\mathcal{C}_i \cap \mathcal{C}_j}(\prod_{k\neq l}(1-\phi_{ij}^k))P_i^l \frac{e_c^\top R_j^\top P_{\beta_l}}{(1-\cos \psi_\mathcal{F})d}(Mv_j^\times\Omega_j-mgR_j^\top e_z)
\end{align}
$$


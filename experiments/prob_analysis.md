
### 確率関数の設計によるアクティブパーセプションの検討と今後の展望

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
P_{ij}^l &= \frac{\beta_l^\top(p_i) R_i e_c -\cos\Psi_\mathcal{F} }{1-\cos\Psi_\mathcal{F}}\frac{\beta_l^\top(p_j) R_j e_c -\cos\Psi_\mathcal{F} }{1-\cos\Psi_\mathcal{F}}
\end{align}
$$
という関数を使用しているが．$P^l_i$が$(p_i, R_i)\in \mathrm{SE(3)}$について微分可能であれば任意の確率関数を設計可能である．今後の目標として，自己位置推定における関数を下限制約することを目的として$P^l_i$を設計することを考える．

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
ここから，新しい確率関数を
$$
\begin{align}
P_{ij}^l &= \exp(-\frac{\sigma^2}{f^2}\,\mathrm{tr}\left[ \frac{P_{\beta_i}}{d_i^2} + \frac{P_{\beta_j}}{d_j^2}\right]^{-1})
\end{align}
$$
のように設計できる．

上記の検討から，関数3によってCBFを設計する場合，下図の様に，CBFはクラメール・ラオの下限を上から抑える関数として機能することがわかる．クラメール・ラオの下限は最尤推定における最適化限界として働くため，理想的な推定アルゴリズムの元では，視野共有を保証するCBFは自己位置推定を行うためのアクティブパーセプションとして捉えることができる．
![[Screenshot 2025-04-14 184806.png]]
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
{P}_{ij}^l = \exp\Bigl(-\frac{\sigma^2}{f^2}\,T\Bigr)
$$
と表せる．$T = \operatorname{tr}(M^{-1})$ として，チェーンルールを適用すると
$$
\dot{{P}}_{ij}^l = -\frac{\sigma^2}{f^2}\,{P}_{ij}^l\,\dot{T}
$$
ここで
$$
\dot{T} = \frac{d}{dt}\operatorname{tr}(M^{-1}) =\operatorname{tr}\Bigl(\frac{d}{dt}(M^{-1})\Bigr) =-\operatorname{tr}\Bigl(M^{-1}\,\dot{M}\,M^{-1}\Bigr)
$$
これを上記の式に代入すると、
$$
\dot{{P}}_{ij}^l = -\frac{\sigma^2}{f^2}\,{P}_{ij}^l\,\Bigl[-\operatorname{tr}(M^{-1}\,\dot{M}\,M^{-1})\Bigr] =\frac{\sigma^2}{f^2}\,{P}_{ij}^l\,\operatorname{tr}(M^{-1}\,\dot{M}\,M^{-1})
$$
よって、
$$
\dot{{P}}_{ij}^l = \frac{\sigma^2}{f^2}\,{P}_{ij}^l\,\operatorname{tr}\Bigl(M^{-1}\,\dot{M}\,M^{-1}\Bigr)
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
\dot{P}_\beta = \frac{P_\beta\,v}{d}\,\beta^\top + \beta\,\frac{v^\top\,P_\beta}{d}
$$
より、各エージェント $i$ について
$$
\frac{d}{dt}\left(\frac{P_{\beta_i}}{d_i^2}\right) =\frac{1}{d_i^2}\left(\frac{P_{\beta_i}\,v_i}{d_i}\,\beta_i^\top + \beta_i\,\frac{v_i^\top\,P_{\beta_i}}{d_i}\right) -\frac{2(-\beta_i^\top v_i)}{d_i^3}\,P_{\beta_i}
$$
すなわち、
$$
\frac{d}{dt}\left(\frac{P_{\beta_i}}{d_i^2}\right) =\frac{P_{\beta_i}\,v_i\,\beta_i^\top + \beta_i\,v_i^\top\,P_{\beta_i}}{d_i^3} +\frac{2(\beta_i^\top v_i)}{d_i^3}\,P_{\beta_i}
$$
ゆえに、
$$
\dot{M} = \frac{P_{\beta_i}\,v_i\,\beta_i^\top + \beta_i\,v_i^\top\,P_{\beta_i} + 2(\beta_i^\top v_i)\,P_{\beta_i}}{d_i^3} +\frac{P_{\beta_j}\,v_j\,\beta_j^\top + \beta_j\,v_j^\top\,P_{\beta_j} + 2(\beta_j^\top v_j)\,P_{\beta_j}}{d_j^3}
$$
以上をまとめると、スカラー化した確率関数
$$
{P}_{ij}^l = \exp\Biggl(-\frac{\sigma^2}{f^2}\,\operatorname{tr}\Bigl\{M^{-1}\Bigr\}\Biggr)
$$
（ただし $M = \frac{P_{\beta_i}}{d_i^2}+\frac{P_{\beta_j}}{d_j^2}$）の時間微分は、
$$
\dot{{P}}_{ij}^l = \frac{\sigma^2}{f^2}\,{P}_{ij}^l\,\operatorname{tr}\Bigl(M^{-1}\,\dot{M}\,M^{-1}\Bigr)
$$
であり、ここで $\dot{M}$ は上記の通り各エージェントの項の和として求まる。
一方，ここで確率関数の問題として，角速度$\Omega_i$に依存しないようになってしまうため，視野内に特徴点を留める関数として機能しない．これは，以下の図のように共有視野境界において関数が非連続になることからも理解できる．

**Cramer-Rao関数**
$$
\begin{align}
P_{ij}^l &= \exp(-\frac{\sigma^2}{f^2}\,\mathrm{tr}\left[ \frac{P_{\beta_i}}{d_i^2} + \frac{P_{\beta_j}}{d_j^2}\right]^{-1})
\end{align}
$$
![[Screenshot from 2025-04-15 18-25-39.png]]
**視野ポテンシャル**
$$
\begin{align}
P_{ij}^l &= \frac{\beta_l^\top(p_i) R_i e_c -\cos\Psi_\mathcal{F} }{1-\cos\Psi_\mathcal{F}}\frac{\beta_l^\top(p_j) R_j e_c -\cos\Psi_\mathcal{F} }{1-\cos\Psi_\mathcal{F}}
\end{align}
$$
![[Screenshot from 2025-04-15 18-25-59.png]]

ここから，上記のCramer-Rao関数と視野ポテンシャルを混合した
Cramer-Rao視野ポテンシャルを考えると下図のようになる．
**Cramer-Rao視野ポテンシャル**
$$
\begin{align}
P_{ij}^l &= \exp(-\frac{\sigma^2}{f^2}\,\mathrm{tr}\left[ \frac{P_{\beta_i}}{d_i^2} + \frac{P_{\beta_j}}{d_j^2}\right]^{-1})\frac{\beta_l^\top(p_i) R_i e_c -\cos\Psi_\mathcal{F} }{1-\cos\Psi_\mathcal{F}}\frac{\beta_l^\top(p_j) R_j e_c -\cos\Psi_\mathcal{F} }{1-\cos\Psi_\mathcal{F}}
\end{align}
$$
![[Screenshot from 2025-04-15 18-25-18.png]]
一方で上記の目的は異なるCBFを最適化問題に張れば良い気もするため上記の様に煩雑な関数を用いる意義は不透明である．
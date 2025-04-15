ここでは，元の行列表現

Pijl=exp⁡(−σ2f2 [Pβidi2+Pβjdj2]−1)P_{ij}^l = \exp\Biggl(-\frac{\sigma^2}{f^2}\,\Bigl[\frac{P_{\beta_i}}{d_i^2}+\frac{P_{\beta_j}}{d_j^2}\Bigr]^{-1}\Biggr)

を，微分しやすいようにスカラー化するため，行列の「トレース」をとる方法を採用します．すなわち，新しいスカラー関数を

Pˉijl=exp⁡(−σ2f2 tr⁡{[Pβidi2+Pβjdj2]−1})\boxed{ \bar{P}_{ij}^l = \exp\Biggl(-\frac{\sigma^2}{f^2}\,\operatorname{tr}\Bigl\{\Bigl[\frac{P_{\beta_i}}{d_i^2}+\frac{P_{\beta_j}}{d_j^2}\Bigr]^{-1}\Bigr\}\Biggr) }

と定義します．ここで，

M≜Pβidi2+Pβjdj2M \triangleq \frac{P_{\beta_i}}{d_i^2}+\frac{P_{\beta_j}}{d_j^2}

とおくと，M−1M^{-1}のトレースを TT と書くと

T=tr⁡(M−1)T = \operatorname{tr}(M^{-1})

となり，関数はより簡潔に

Pˉijl=exp⁡(−σ2f2 T)\bar{P}_{ij}^l = \exp\Bigl(-\frac{\sigma^2}{f^2}\,T\Bigr)

と表せます。

以下，このスカラー関数の時間微分を段階的に計算します。

---

## 1. チェーンルールによる微分

まず，T=tr⁡(M−1)T = \operatorname{tr}(M^{-1}) として，チェーンルールを適用すると，

Pˉ˙ijl=ddtexp⁡(−σ2f2 T)=exp⁡(−σ2f2 T)(−σ2f2)T˙.\dot{\bar{P}}_{ij}^l = \frac{d}{dt}\exp\Bigl(-\frac{\sigma^2}{f^2}\,T\Bigr) =\exp\Bigl(-\frac{\sigma^2}{f^2}\,T\Bigr)\left(-\frac{\sigma^2}{f^2}\right)\dot{T}.

すなわち

Pˉ˙ijl=−σ2f2 Pˉijl T˙ .\boxed{ \dot{\bar{P}}_{ij}^l = -\frac{\sigma^2}{f^2}\,\bar{P}_{ij}^l\,\dot{T}\,. }

---

## 2. TT の時間微分 T˙\dot{T} の計算

行列 MM の逆行列の時間微分は，

ddt(M−1)=− M−1 M˙ M−1\frac{d}{dt}(M^{-1}) = -\,M^{-1}\,\dot{M}\,M^{-1}

という公式が成り立ります．よって，

T˙=ddttr⁡(M−1)=tr⁡(ddt(M−1))=−tr⁡(M−1 M˙ M−1).\dot{T} = \frac{d}{dt}\operatorname{tr}(M^{-1}) =\operatorname{tr}\Bigl(\frac{d}{dt}(M^{-1})\Bigr) =-\operatorname{tr}\Bigl(M^{-1}\,\dot{M}\,M^{-1}\Bigr).

これを上記の式に代入すると，

Pˉ˙ijl=−σ2f2 Pˉijl [−tr⁡(M−1 M˙ M−1)]=σ2f2 Pˉijl tr⁡(M−1 M˙ M−1) .\dot{\bar{P}}_{ij}^l = -\frac{\sigma^2}{f^2}\,\bar{P}_{ij}^l\,\Bigl[-\operatorname{tr}(M^{-1}\,\dot{M}\,M^{-1})\Bigr] =\frac{\sigma^2}{f^2}\,\bar{P}_{ij}^l\,\operatorname{tr}(M^{-1}\,\dot{M}\,M^{-1})\,.

よって，

Pˉ˙ijl=σ2f2 Pˉijl tr⁡(M−1 M˙ M−1).\boxed{ \dot{\bar{P}}_{ij}^l = \frac{\sigma^2}{f^2}\,\bar{P}_{ij}^l\,\operatorname{tr}\Bigl(M^{-1}\,\dot{M}\,M^{-1}\Bigr). }

---

## 3. MM の時間微分 M˙\dot{M} の計算

ここで，MM は各エージェント i,ji,j について

M=Pβidi2+Pβjdj2M = \frac{P_{\beta_i}}{d_i^2} + \frac{P_{\beta_j}}{d_j^2}

と定義されています．各項に対して微分を行います．一般に，スカラー関数 dd と行列 PβP_\beta の組合せの微分は，

ddt(Pβd2)=1d2P˙β−2d˙d3 Pβ.\frac{d}{dt}\left(\frac{P_\beta}{d^2}\right) =\frac{1}{d^2}\dot{P}_\beta - \frac{2\dot{d}}{d^3}\,P_\beta.

また，Pβ=I−β β⊤P_\beta = I - \beta\,\beta^\topであるため，

P˙β=−β˙ β⊤−β β˙⊤.\dot{P}_\beta = -\dot{\beta}\,\beta^\top - \beta\,\dot{\beta}^\top.

与えられている関係式

β˙=−Pβd v,d˙=−β⊤v\dot{\beta} = -\frac{P_\beta}{d}\,v,\qquad \dot{d} = -\beta^\top v

を用いると，

P˙β=Pβ vd β⊤+β v⊤ Pβd .\dot{P}_\beta = \frac{P_\beta\,v}{d}\,\beta^\top + \beta\,\frac{v^\top\,P_\beta}{d}\,.

したがって，各エージェント ii については

ddt(Pβidi2)=1di2(Pβi vidi βi⊤+βi vi⊤ Pβidi)−2(−βi⊤vi)di3 Pβi .\frac{d}{dt}\left(\frac{P_{\beta_i}}{d_i^2}\right) =\frac{1}{d_i^2}\left(\frac{P_{\beta_i}\,v_i}{d_i}\,\beta_i^\top + \beta_i\,\frac{v_i^\top\,P_{\beta_i}}{d_i}\right) -\frac{2(-\beta_i^\top v_i)}{d_i^3}\,P_{\beta_i}\,.

すなわち，

ddt(Pβidi2)=Pβi vi βi⊤+βi vi⊤ Pβidi3+2(βi⊤vi)di3 Pβi .\frac{d}{dt}\left(\frac{P_{\beta_i}}{d_i^2}\right) =\frac{P_{\beta_i}\,v_i\,\beta_i^\top + \beta_i\,v_i^\top\,P_{\beta_i}}{d_i^3} +\frac{2(\beta_i^\top v_i)}{d_i^3}\,P_{\beta_i}\,.

同様に，エージェント jj についても

ddt(Pβjdj2)=Pβj vj βj⊤+βj vj⊤ Pβjdj3+2(βj⊤vj)dj3 Pβj .\frac{d}{dt}\left(\frac{P_{\beta_j}}{d_j^2}\right) =\frac{P_{\beta_j}\,v_j\,\beta_j^\top + \beta_j\,v_j^\top\,P_{\beta_j}}{d_j^3} +\frac{2(\beta_j^\top v_j)}{d_j^3}\,P_{\beta_j}\,.

ゆえに，

M˙=Pβi vi βi⊤+βi vi⊤ Pβi+2(βi⊤vi) Pβidi3+Pβj vj βj⊤+βj vj⊤ Pβj+2(βj⊤vj) Pβjdj3 .\boxed{ \dot{M} = \frac{P_{\beta_i}\,v_i\,\beta_i^\top + \beta_i\,v_i^\top\,P_{\beta_i} + 2(\beta_i^\top v_i)\,P_{\beta_i}}{d_i^3} +\frac{P_{\beta_j}\,v_j\,\beta_j^\top + \beta_j\,v_j^\top\,P_{\beta_j} + 2(\beta_j^\top v_j)\,P_{\beta_j}}{d_j^3}\,. }

---

## 4. 最終的な結果

以上をまとめると，スカラー化した確率関数

Pˉijl=exp⁡(−σ2f2 tr⁡{M−1})\bar{P}_{ij}^l = \exp\Biggl(-\frac{\sigma^2}{f^2}\,\operatorname{tr}\Bigl\{M^{-1}\Bigr\}\Biggr)

（ただし M=Pβidi2+Pβjdj2M = \frac{P_{\beta_i}}{d_i^2}+\frac{P_{\beta_j}}{d_j^2}）の時間微分は、

Pˉ˙ijl=σ2f2 Pˉijl tr⁡(M−1 M˙ M−1)\boxed{ \dot{\bar{P}}_{ij}^l = \frac{\sigma^2}{f^2}\,\bar{P}_{ij}^l\,\operatorname{tr}\Bigl(M^{-1}\,\dot{M}\,M^{-1}\Bigr) }

であり，ここで

M˙=Pβi vi βi⊤+βi vi⊤ Pβi+2(βi⊤vi) Pβidi3+Pβj vj βj⊤+βj vj⊤ Pβj+2(βj⊤vj) Pβjdj3 .\dot{M} = \frac{P_{\beta_i}\,v_i\,\beta_i^\top + \beta_i\,v_i^\top\,P_{\beta_i} + 2(\beta_i^\top v_i)\,P_{\beta_i}}{d_i^3} +\frac{P_{\beta_j}\,v_j\,\beta_j^\top + \beta_j\,v_j^\top\,P_{\beta_j} + 2(\beta_j^\top v_j)\,P_{\beta_j}}{d_j^3}\,.

この形では，関数自体がスカラーとなっており，チェーンルールおよび行列微分の公式を用いて比較的扱いやすい形で時間微分が記述できています。

---

## まとめ

1. **スカラー化:**  
    Pˉijl=exp⁡(−σ2f2 tr⁡(M−1))\bar{P}_{ij}^l = \exp\Bigl(-\frac{\sigma^2}{f^2}\,\operatorname{tr}(M^{-1})\Bigr)  
    M=Pβidi2+Pβjdj2M = \frac{P_{\beta_i}}{d_i^2}+\frac{P_{\beta_j}}{d_j^2}
    
2. **時間微分:**  
    Pˉ˙ijl=σ2f2 Pˉijl tr⁡(M−1 M˙ M−1)\dot{\bar{P}}_{ij}^l = \frac{\sigma^2}{f^2}\,\bar{P}_{ij}^l\,\operatorname{tr}(M^{-1}\,\dot{M}\,M^{-1})
    
3. **MM の時間微分:**  
    M˙\dot{M} は上記の通り，各エージェントの項の和として求まる．
    

この結果により，対象の確率関数がスカラー値として定義され，さらにその時間変化が解析的に評価できる形となりました。
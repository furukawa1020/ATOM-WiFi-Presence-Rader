# ISS-009 キャリブレーションモデル学習

## データ分離

特徴量は、収録前に次のいずれかの区分を持つランへ登録する。フレームを収録後にランダム分割する処理は持たない。

- Training: 特徴量mean/scaleと2つのロジスティック重みの学習だけに使う。
- ThresholdTuning: 在室閾値と動作閾値の探索だけに使う。
- FinalValidation: 重みと閾値を確定した後の評価だけに使う。

各ランはセッションID、ラベル、反復番号、時間ブロックを保持する。同じセッションIDは二度登録できないため、区分をまたぐデータ漏洩を防止できる。

## 教師値

標準ラベルの教師値は次のとおり。

| ラベル | 在室 | 動作 |
| --- | --- | --- |
| Empty | Negative | Negative |
| Still | Positive | Negative |
| Motion | Positive | Positive |
| Nuisance | 明示指定 | 明示指定 |

Nuisanceは現象に応じてPositive、Negative、Ignoreを個別指定する。両方がIgnoreのサンプルは学習にも評価にも寄与しないため受け付けない。

## 学習

最大16特徴、24ラン、512サンプルを固定長領域へ保持する。正規化統計はTrainingだけから算出し、クラス重み付きバッチ勾配降下で在室用と動作用のモデルを別々に学習する。

閾値はThresholdTuning上で次の目的関数が最大になる組み合わせを探索する。

0.35 * occupied recall + 0.25 * motion recall + 0.20 * precision + 0.15 * balanced accuracy - 0.05 * false alarm rate

precision、balanced accuracy、false alarm rateは在室ヘッドと動作ヘッドの両方を反映する。同点の場合は誤警報を抑制しやすい高い閾値を選ぶ。

FinalValidationは閾値確定後に一度だけ評価され、その結果はモデル選択へ戻さない。各区分で在室・動作それぞれの正例と負例が揃わない場合、モデルをreadyにしない。

## 後続PR

DetectionFeaturesから固定順序の特徴ベクトルを作るアダプター、既存LogisticClassifierへの適用、モデルのCRC付きNVS永続化は後続PRで実装する。

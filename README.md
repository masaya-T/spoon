# spoon

スプーンの持ち手の傾きに合わせて、スプーンの先端がモータによって回転するシステム。
傾きは加速度センサーによって検知し、その傾き分モータを回転させ水平を保つ。
１次遅れ系のフィードバック制御によって、センサ値を安定させている。

# DEMO
制御量に対してフィルタを通した場合（青）とそうでない場合（赤）
![glaph](./glaph.png "glaph")


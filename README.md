**システム要件**
Ubuntu 24.04 LTS
ROS2 jazzy
NAV2

**それぞれのファイルの説明**

.env
    このファイルはAPIkeyが入っているだけです

app.py
    ユーザーとの対話をしながらメニューを決めます
    そして決めたものをjson形式の英語テキスト群にしてそれをROSに送信します
    現状で定義されている物はにんじん、たまねぎ、じゃがいも、カレールー、お肉です
    そして最後にロボットに送信を押すことでsimple_navigatorに送信される

**起動方法**
ターミナルを3つ使って起動&作動させます

    ターミナル1
    ここではGazebo(仮想でロボットを用意して動かすやつ),liviz(ロボットがどうやって世界を認識しているかを確認するアプリ)を起動させます
    この順番でコマンドを打ちます

    source /opt/ros/jazzy/setup.bash
    export TURTLEBOT3_MODEL=bueger
    export ROS_DOMEIN_ID=0
    ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False

    ターミナル2
    source /opt/ros/jazzy/setup.bash
    export ROS_DOMEIN_ID=0
    cd (自分の環境のこれらが入っているフォルダのパス)
    python3 simple_navigator.py

    ターミナル3
    source /opt/ros/jazzy/setup.bash
    export ROS_DOMEIN_ID=0
    source venv/bin/activate
    cd (自分の環境のこれらが入っているフォルダのパス)
    streamlit run app.py
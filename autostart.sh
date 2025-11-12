#!/bin/bash

# 初始启动函数
start_can() {
    echo "正在启动can0接口..."
    sudo slcand -o -c -s6 /dev/CAN0 can0
    sudo ifconfig can0 up
    
    # 等待接口稳定
    sleep 2
}

# 检查can0状态的函数
check_can_status() {
    local can_status=$(ip -details link show can0 2>/dev/null)
    
    # 检查接口是否存在
    if [[ -z "$can_status" ]]; then
        echo "can0接口不存在"
        return 1
    fi
    
    # 检查接口状态是否为UP
    if ! echo "$can_status" | grep -q "state UP"; then
        echo "can0接口存在但状态为DOWN"
        return 1
    fi
    
    echo "can0接口已正常启动(UP)"
    return 0
}

# 主启动流程
start_can

# 检查初始启动状态
if ! check_can_status; then
    echo "can0接口启动失败，正在重试..."
    exec "$0"
fi

# 后台监控进程
(
    while true; do
        sleep 30
        if ! check_can_status; then
            echo "检测到can0异常，正在重启..."
            
            # 先清理可能的残留进程
            sudo pkill -f "slcand.*can0"
            sleep 1
            
            # 重新启动
            start_can
            
            # 验证重启是否成功
            if check_can_status; then
                echo "can0重启成功"
            else
                echo "can0重启失败"
            fi
        fi
    done
) &

echo "can0监控程序已启动"
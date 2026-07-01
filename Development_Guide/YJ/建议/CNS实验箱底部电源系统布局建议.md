# CNS 实验箱底部电源系统布局建议

本文件用于指导 CNS 实验箱底部电源系统装配。当前结构采用上下两层设计：上层放开发板、传感器、屏幕、电调和电机等功能器件；底层只放电源、保护、分配、采样和穿层接口。箱体右侧开孔用于第一电池充电/维护接口和第二电池 XT60 动力接入。

## 1. 底层俯视布局图

下图按箱体实际左右方向绘制，采用“左侧弱电、中部穿层、右侧动力、下侧单点共地”的区间模块布局。连接线按用途分开：弱电供电线、动力供电线、采样线、控制线和地线尽量走独立线槽，避免互相穿插。

```mermaid
flowchart LR
    subgraph BOX["CNS 实验箱底层电源系统俯视布局"]
        direction TB

        subgraph MAIN["主体模块区：左弱电 / 中采样穿层 / 右动力"]
            direction LR

            subgraph LEFT["左侧：第一电池与稳压区"]
                direction TB
                BAT1["第一电池\n箱体内部"]:::battery
                FUSE1["主控保险"]:::protect
                METER1["第一电池电流计\n采集第一电池电压/电流"]:::meter
                SW1["主控开关 / 防反接"]:::protect
                REG["稳压板\n给开发板和低压模块供电"]:::board

                BAT1 --> FUSE1 --> METER1 --> SW1 --> REG
            end

            subgraph MID["中部：开发板采样与穿层区"]
                direction TB
                DEV_ADC["开发板 ADC / GND 采样汇入口"]:::sample
                LOW_UP["弱电供电上翻线束"]:::conn
                SAMPLE_UP["电压/电流采样线束"]:::sample
                CTRL_UP["电调控制线束"]:::sample
                MOTOR_UP["动力上翻线束"]:::motor
            end

            subgraph RIGHT["右侧：外接动力与分电区"]
                direction TB
                CHG1["第一电池充电/维护口\n右侧开孔"]:::conn
                XT60["右侧 XT60\n第二电池接入口"]:::battery
                FUSE2["动力保险"]:::protect
                METER2["第二电池电流计\n采集第二电池电压/电流"]:::meter
                PDB["分电板\n只负责电机动力分配"]:::motor
                ESC["上层电调 / 电机"]:::motor

                XT60 --> FUSE2 --> METER2 --> PDB --> MOTOR_UP --> ESC
            end
        end

        subgraph GNDBUS["下侧地线区：单点共地"]
            direction LR
            STAR["单点共地端子"]:::ground
        end
    end

    REG --> LOW_UP
    LOW_UP --> DEV_ADC

    METER1 -. "第一电池电压信号\n第一电池电流信号\n第一电流计信号地" .-> SAMPLE_UP
    METER2 -. "第二电池电压信号\n第二电池电流信号\n第二电流计信号地" .-> SAMPLE_UP
    SAMPLE_UP --> DEV_ADC

    DEV_ADC -. "开发板 GND" .-> STAR

    CTRL_UP -. "控制信号\n不承载动力电流" .-> ESC

    REG -. "弱电地" .-> STAR
    PDB -. "动力地\n大电流回流不经过开发板" .-> STAR

    classDef battery fill:#fff3cd,stroke:#b8860b,stroke-width:2px,color:#111;
    classDef protect fill:#fff8d6,stroke:#c28b00,stroke-width:2px,color:#111;
    classDef meter fill:#e8f5ff,stroke:#1976a2,stroke-width:2px,color:#111;
    classDef board fill:#e8f5ff,stroke:#1976a2,stroke-width:2px,color:#111;
    classDef conn fill:#eefbea,stroke:#2e7d32,stroke-width:2px,color:#111;
    classDef sample fill:#f3e8ff,stroke:#7e57c2,stroke-width:2px,color:#111;
    classDef ground fill:#eeeeee,stroke:#45515c,stroke-width:2px,color:#111;
    classDef motor fill:#fff0e6,stroke:#c6531a,stroke-width:2px,color:#111;
```

## 2. 布局重点

- 左右方向按箱体实际方向绘制：左侧为内置第一电池与稳压区，右侧为外接动力电池与分电区。
- 右侧开孔承担两个功能：第一电池充电/维护，第二电池 XT60 动力接入。
- 第一电池在箱体内部，第一电池与稳压板之间串接第一电池电流计。
- 第二电池为外接动力电池，进入箱体后先经过动力保险，再经过第二电池电流计，最后到分电板。
- 电流计同时负责电压、电流采样，其采样信号和信号地进入开发板 ADC/GND 采样入口。
- 稳压板只负责弱电供电；分电板只负责电机动力分配。两者之间不直接连接电源线、采样线或控制线。
- 开发板 GND、稳压板 GND、分电板 GND 最终汇到单点共地；动力大电流回流不得经过开发板、传感器或弱电线束。
- 弱电供电线、动力供电线、采样线、控制线和地线应分线槽固定，尽量平行分层走线，避免交叉缠绕。
- 穿层线束需要固定，穿孔处加护线圈，避免箱体边缘磨破绝缘皮。

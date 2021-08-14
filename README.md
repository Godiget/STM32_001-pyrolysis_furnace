# STM32_001-f103-MAP-LCD

|                         Siemens MAP                          |                         Mercedes MAP                         |
| :----------------------------------------------------------: | :----------------------------------------------------------: |
| <img src="https://github.com/Godiget/STM32_001-f103-MAP-LCD/blob/master/fig/map_vdo.jpg?raw=true" style="zoom:50%;" /> | <img src="https://github.com/Godiget/STM32_001-f103-MAP-LCD/blob/master/fig/map_mercedes.jpg?raw=true" style="zoom:50%;" /> |

___

#### :pill: s​tm32f103 "bluepill" pinout:

![](https://github.com/Godiget/STM32_001-f103-MAP-LCD/blob/master/fig/stm32f103c8t6_pinout.png?raw=true)

___

#### NTC Термистор:

<img src="https://github.com/Godiget/STM32_001-f103-MAP-LCD/blob/master/fig/ntc_thermistor.jpg?raw=true" style="zoom:50%;" />

- Номинальное напряжение, V 3,4(±0,3)
- Сопротивление при 15°С, Ом 4033…4838
- Сопротивление при 128°С, Ом 76,7…85,1
- Выход напряжения при 15°С, % 92,1…93,3
- Выход напряжения при 128°С, % 18,1…19,7
- М12x1.5

___

#### Дроссельная заслонка:

<img src="https://github.com/Godiget/STM32_001-f103-MAP-LCD/blob/master/fig/TPS.png?raw=true" style="zoom: 33%;" />

___

##### Пример подключения математических функций для STM32F4 в stm32cubeide

1) Открываем Project -> Properties -> C++ general -> Paths and Symbols
2) На вкладке Libraries добавляем название библиотеки libarm_cortexM3l_math.a, без расширения ".a" и убрав буквы "lib": т.е. arm_cortexM3l_math
3) На вкладке Library Paths добавляем путь к библиотеке libarm_cortexM3l_math.a
"C:\Documents and Settings\имя_пользователя\STM32Cube\Repository\STM32Cube_FW_F3_версия\Drivers\CMSIS\Lib\GCC"
4) На вкладке Symbols добавляем два значения: __FPU_PRESENT (спереди два символа подчеркивания) , ARM_MATH_CM3 (тут в зависимости от серии микроконтроллера)
В текущей версии также придётся вручную скопировать в проект файлы arm_math.h, const_structs.h и arm_common_tables.h из
"C:\Documents and Settings\имя_пользователя\STM32Cube\Repository\STM32Cube_FW_F3_версия\Drivers\CMSIS\DSP\Include".

___

### Формат сообщений RS485

| :left_right_arrow: | Описание пакета           | Формат пакета                                                |
| ------------------ | ------------------------- | ------------------------------------------------------------ |
| :arrow_right:      | запрос данных             | [0, *, *, *]                                                 |
| :arrow_right:      | установка заслонок        | [3, spTP1, spTP2, *]                                         |
| :arrow_right:      | установка параметров PID1 | [4, Kp, Ki, Kd]                                              |
| :arrow_right:      | установка параметров PID2 | [5, Kp, Ki, Kd]                                              |
| :arrow_left:       | ответ на запрос           | [error_code, TP1, TP2, spTP1, spTP2, ntc1, ntc2, ntc3, ntc4, TC] |

___

### Формат сообщений CAN

| :left_right_arrow: | Описание пакета           | CAN ID | Формат пакета                          |
| ------------------ | ------------------------- | ------ | -------------------------------------- |
| :arrow_right:      | установка заслонок        | 0x0099 | [spTP1, spTP2]                         |
| :arrow_left:       | положение заслонок        | 0x0100 | [TP1_lb, TP1_hb, TP2_lb, TP2_hb]       |
| :arrow_right:      | установка параметров PID1 | 0x0111 | [Kp, Ki, Kd]                           |
| :arrow_right:      | установка параметров PID2 | 0x0112 | [Kp, Ki, Kd]                           |
| :arrow_left:       | температура ntc + TC      | 0x0120 | [ntc1, ntc2, ntc3, ntc4, TC_lb, TC_hb] |


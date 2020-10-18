# STM32_001-f103-MAP-LCD

Siemens MAP pinout:
![](https://github.com/Godiget/STM32_001-f103-MAP-LCD/blob/master/fig/map_vdo.jpg)

Mercedes MAP pinout:
![](https://github.com/Godiget/STM32_001-f103-MAP-LCD/blob/master/fig/map_mercedes.jpg)

f103 pinout:
![](https://github.com/Godiget/STM32_001-f103-MAP-LCD/blob/master/fig/stm32f103c8t6_pinout.png)

NTC Термистор:
![](https://github.com/Godiget/STM32_001-f103-MAP-LCD/blob/master/fig/ntc_thermistor.jpg)

- Номинальное напряжение, V 3,4(±0,3)
- Сопротивление при 15°С, Ом 4033…4838
- Сопротивление при 128°С, Ом 76,7…85,1
- Выход напряжения при 15°С, % 92,1…93,3
- Выход напряжения при 128°С, % 18,1…19,7
- Размер под ключ S19
- Резьба М12x1.5
- Масса, кг 0,044

Дроссельная заслонка:
![](https://github.com/Godiget/STM32_001-f103-MAP-LCD/blob/master/fig/TPS.png)

Пример подключения математических функций для STM32F4 в stm32cubeide
1) Открываем Project -> Properties -> C++ general -> Paths and Symbols
2) На вкладке Libraries добавляем название библиотеки libarm_cortexM4lf_math.a, без расширения ".a" и убрав буквы "lib": т.е. arm_cortexM4lf_math
3) На вкладке Library Paths добавляем путь к библиотеке libarm_cortexM4lf_math.a
"C:\Documents and Settings\имя_пользователя\STM32Cube\Repository\STM32Cube_FW_F4_версия\Drivers\CMSIS\Lib\ARM"
4) На вкладке Symbols добавляем два значения: __FPU_PRESENT (спереди два символа подчеркивания) , ARM_MATH_CM4 (тут в зависимости от серии микроконтроллера)
В текущей версии также придётся вручную скопировать в проект файлы arm_math.h, const_structs.h и arm_common_tables.h из
"C:\Documents and Settings\имя_пользователя\STM32Cube\Repository\STM32Cube_FW_F4_версия\Drivers\CMSIS\DSP\Include".
# Ballistics Calculator 🔭


## Описание

**Ballistics Calculator** - это высокопроизводительный C++ инструмент для расчета баллистики, траекторий и анализа полета снарядов. Программа использует физические формулы для решения баллистических задач в научных целях и визуализации.

### Основные возможности:
- ⚡ **Высокая производительность** - оптимизированные C++20 алгоритмы
- 🔍 **Обратные задачи** - поиск параметров по известным характеристикам
- 🎯 **Точность** - использование современных математических библиотек
- 📈 **Бенчмаркинг** - измерение производительности расчетов

## Примеры использования

### Базовый расчет траектории
```cpp
#include "BallisticSolver.h"

int main() {
    BallisticSolver solver;
    Parametrs params;
    
    params.start_velocity = 100.0;    // м/с
    params.throwing_angle_degrees = 45.0;
    params.start_acceleration = 9.81;
    
    auto result = solver.solve_trajectory(params);
    
    std::cout << "Дальность: " << result.max_distance << " м\n";
    std::cout << "Макс. высота: " << result.max_height << " м\n";
    std::cout << "Время полета: " << result.total_time << " с\n";
    
    return 0;
}

```
# Установка и использование

## Требования
- Visual Studio 2019 или новее
- Поддержка стандарта C++20
- Windows 10/11

## Инструкция по сборке

### Откройте проект в Visual Studio:
1. Запустите Visual Studio
2. Выберите "Open a project or solution"
3. Найдите и откройте файл `BallisticsCalculator.sln`

### Настройте конфигурацию сборки:
- В верхней панели выберите `Debug` или `Release`
- Убедитесь, что выбрана платформа `x64`

### Соберите проект:
- Нажмите `Build` → `Build Solution` (Ctrl+Shift+B)
- Или используйте F7

### Если не собралось:
1)ПКМ на проект "Ballistic_Calculation" в "Обозревателе решений"
2)Свойства
3)В меню "свойста конфигурации" - подпункт "каталоги VC++"
4)"Включаемые каталоги" -> "изменить"
<img width="794" height="535" alt="image" src="https://github.com/user-attachments/assets/6a4892e5-e2cd-4139-b378-84a9f4d44a6d" />

5)Добавить папки проекта (создать строку -> ... -> выбрать в проводнике папку):
<img width="1271" height="540" alt="image" src="https://github.com/user-attachments/assets/97ba08cf-88a2-4c2c-ac9b-4d717d2cf05b" />
<img width="520" height="343" alt="image" src="https://github.com/user-attachments/assets/8508ddf4-91cd-4da2-b04c-81f241c848ca" />
<img width="470" height="347" alt="image" src="https://github.com/user-attachments/assets/3a17e622-69c1-42cd-a4bc-a43a5a5a8ed3" />
подключить все 3 папки (и добавлять другие папки с кодом при их создании вами)
<img width="612" height="109" alt="image" src="https://github.com/user-attachments/assets/4da32af8-8db5-4bed-9780-083fa8288987" />




### Запустите программу:
- Нажмите `Debug` → `Start Without Debugging` (Ctrl+F5)

## Структура проекта

```
BallisticsCalculator/
├── BallisticSolver.h/cpp    # Основной решатель баллистических задач
├── Parametrs.h/cpp          # Структура параметров и данных
├── BalisticTester.h/cpp     # Система тестирования и верификации
├── main.cpp                 # Точка входа в программу
└── BallisticsCalculator.sln # Файл решения Visual Studio
```

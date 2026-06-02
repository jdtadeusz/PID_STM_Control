# PID_STM_Control: System Lewitacji Aerodynamicznej

![Status: Działa](https://img.shields.io/badge/Status-Dzia%C5%82a-success)
![Język: C](https://img.shields.io/badge/J%C4%99zyk-C-blue)
![Platforma: STM32](https://img.shields.io/badge/Platforma-STM32-orange)

<img width="260" height="462" alt="PID_Stm32" src="https://github.com/user-attachments/assets/bed42bfd-7802-4ef6-9be8-a41be049812e" />


## Opis Projektu

PID_STM_Control to projekt embedded demonstrujący nieliniową regulację PID na mikrokontrolerze STM32. Celem układu jest precyzyjna, stabilna lewitacja bardzo lekkiej piłeczki (ok. 3g) wewnątrz 35-milimetrowej rury aerodynamicznej przy wykorzystaniu przemysłowego wentylatora sterowanego sygnałem PWM.

Projekt rozwiązuje szereg rzeczywistych problemów inżynierskich, takich jak szum optyczny czujników, asymetria grawitacyjna czy nieliniowa dynamika płynów.

## Rozwiązania Inżynierskie i Funkcjonalności

Układ sterowania został napisany od zera i zawiera mechanizmy spotykane w przemysłowej automatyce:

* **Autorski, Równoległy Kontroler PID:** Własna implementacja algorytmu z konfigurowalnym czasem próbkowania (`dt`).
* **Dynamiczny Feedforward (Gain Scheduling):** Adaptacyjna baza PWM kompensująca nieliniowość aerodynamiczną. Układ automatycznie zmienia punkt pracy w zależności od zadanego Setpointu (inna moc potrzebna na dole, inna na górze rury).
* **Zero-Crossing Anti-Windup:** Mechanizm błyskawicznego resetowania członu całkującego w momencie przecięcia linii błędu, zapobiegający potężnym, opóźnionym przeregulowaniom.
* **Asymetryczne Limity Nasycenia:** Oddzielne, dynamicznie strojone granice dla hamowania (spadek piłki) i przyspieszania (wznoszenie), chroniące przed uderzeniami obiektu o czujnik.
* **Filtracja EMA (Exponential Moving Average):** Lekki, jednobiegunowy filtr dolnoprzepustowy niwelujący sprzętowy szum lasera ToF, zapewniający gładki sygnał dla członu różniczkującego ($K_d$).

## Architektura i Sprzęt

* **Mikrokontroler:** STM32 (konfiguracja przez HAL)
* **Aktuator:** Wentylator przemysłowy - ARCTIC S8038-10K z wbudowanym sterownikiem (sterowanie Timer PWM bezpośrednio do rejestrów)
* **Czujnik:** Laserowy sensor odległości Time-of-Flight VL53L0X (komunikacja I2C, odczyt Continuous Fast)
* **Telemetria:** Dwukierunkowa komunikacja UART z komputerem PC.

## Telemetria w Czasie Rzeczywistym

Do projektu dołączony jest autorski skrypt w języku Python (`plot_pid.py`) wykorzystujący bibliotekę `matplotlib`. Narzędzie to działa jak cyfrowy oscyloskop, na bieżąco odbierając logi z mikrokontrolera przez port szeregowy i wizualizując:
* Aktualną pozycję piłeczki na tle zadanego Setpointu.
* Surowy sygnał sterujący PWM oraz wypracowaną korektę PID.
* Dokładne wartości przesyłane co 50ms w interfejsie graficznym.

## Technologie
* C / STM32 HAL API
* I2C / PWM / UART
* Python 3 (PySerial, Matplotlib, Regex)

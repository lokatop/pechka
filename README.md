# pechka

Оборудование: Энкордер, liquid жк экран, arduino (mega или uno), и нагревательные элементы

Задача: сделать так, чтобы был интерфейс(чтобы мы могли добавить температуру(max), время, длительность, шаги нагрева.
И автоматический подбор PID параметров(которое можно выбрать в меню с помощью энкодера)

Решение в файле 05-11.ino
Присутствуют некоторые проблемы связанные с перегреванием(или слишком быстрым нагревом элементов) 
в связи с чем сложно сделать правильную выборку необходмиую для расчета PID. 

Было выполнена возможность держать на нужной температуре наши нагревательные элементы двумя подходами(описано в коде)
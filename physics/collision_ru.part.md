---
layout: default
title: "Робототехника"
description: "Физика столкновений на примере свободного физического движка реального времени Эрвина Куманса Bullet"
date: 
---


### 5.3.2 Пересекающиеся Луч или Сегмент и Сфера

Пусть луч задается формулой $R (t)=P + t \textbf d, t ≥ 0$, где P - начало луча и нормализованный вектор направления $\textbf d$, $\textbf ||d||=1$. Если $R(t)$ описывает сегмент, а не луч, то $0 ≤ t ≤ t_{max}$. Пусть граница сферы определяется как $(X - C) · (X - C)=r^2$, где $C$ - центр сферы, а $r$ - ее радиус. Чтобы найти значение $t$, при котором луч пересекает поверхность сферы, $R (t)$ подставляется вместо $X$, давая

$$(P + t \textbf d − C) · (P + t \textbf d − C) = r^2 .$$

Пусть $\textbf m = P − C$, тогда:

- $(\textbf m + t \textbf d) · (\textbf m + t \textbf d) = r^2$ ⇔ (замена $\textbf m = P − C$)
- $(\textbf d · \textbf d)t^2 + 2(\textbf +m · \textbf d)t + (\textbf m · \textbf m) = r^2$ ⇔ (раскрытие скалярного произведения)
- $t^2 + 2(\textbf m · \textbf d)t + (\textbf m · \textbf m) − r^2 = 0$  (упрощение $\textbf d · \textbf d = 1$; каноническая форма квадратного уравнения)

Это квадратное уравнение в t. Для квадратичной формулы $t^2 + 2bt + c = 0$, решения даются $t = −b ± \sqrt {b^2 − c}$. Here, b = m · d and c = (m · m) − r 2 .

Solving the quadratic has three outcomes, categorized by the discriminant d =
b2 − c. If d < 0, there are no real roots, which corresponds to the ray missing the
sphere completely. If d = 0, there is one real (double) root, corresponding to the
ray hitting the sphere tangentially in a point. If d > 0, there are two real roots
and the ray intersects the sphere twice: once entering and once leaving the sphere
boundary. In the
√ latter case, the smaller intersection t value is the relevant one, given
by t = −b − b2 − c. However, it is important to distinguish the false intersection
case of the ray starting outside the sphere and pointing away from it, resulting in an

### 5.3.3 Пересекающиеся Луч или Сегмент и Параллелипипед
### 5.3.4 Пересекающиеся Линия и Треугольник
### 5.3.5 Пересекающиеся Линия и Четырехугольник
### 5.3.6 Пересекающиеся Луч или Сегмент и Треугольник
### 5.3.7 Пересекающиеся Луч или Сегмент и Цилиндр
### 5.3.8 Пересекающиеся Луч или Сегмент и Выпуклый многогранник
## 5.4 Дополнительные тесты
### 5.4.1 Тестирование Точки в многоугольнике
### 5.4.2 Тестирование Точки в Треугольнике
### 5.4.3 Тестирование Точки в Многограннике
### 5.4.4 Пересечение двух Плоскостей
### 5.4.5 Пересечение трех Плоскостей
## 5.5 Тест динамических пересечений
### 5.5.1 Уменьшение вдвое интервала пересечения движущихся объектов
### 5.5.2 Тест разделяющей оси для движущихся выпуклых объектов
### 5.5.3 Пересечение движущейся Сферы относительно Плоскости
### 5.5.4 Пересечение движущихся AABB относительно Плоскости
### 5.5.5 Пересечение движущейся Сферы относительно Сферы
### 5.5.6 Пересечение движущейся сферы относительно треугольника (и многоугольника)

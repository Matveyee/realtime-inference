# REALTIME-INFERENCE

Этот проект реализует высокопроизводительный пайплайн по распознаванию объектов с камеры в реальном времени на базе процессоров Rockchip.

## Структура

```text
complete_examples/
 ---- benckmark_inference_hailo/        Тест производительности нейросети Hailo (на ускорителях Hailo)           
 ---- benchmark_inference_rknn/         Тест производительности нейросети RKNN ( на чипе Rockchip)
include/
 ---- hailo/                            Заголочные файлы для hailort
 ---- inc/                              Заголочные файлы для Rockchip MPP
 ---- libyuv/                           Библиотека для конвертации изображений на CPU
 ---- rga/                              Заголовочные файлы для библиотеки Rockchip RGA для аппаратной конвертации изображений
 ...                                    Заголовочные файлы проекта
libs/
 ---- GxGVTL.cti                        Библиотека для камер Galaxy
 ---- GxU3VTL.cti                       Библиотека для камер Galaxy
 ---- libgxapi.so                       Библиотека для камер Galaxy
 ---- libhailort.so                     Библиотека hailort для использования ускорителей Hailo
 ---- libhailort.so.4.23.0              Библиотека hailort для использования ускорителей Hailo
 ---- librga.so                         Библиотека Rockchip RGA
 ---- librknnrt.so                      Библиотека RKNN
 ---- librockchip_mpp.so                Библиотека Rockchip MPP для аппаратного декодирования видео
 ---- libyuv.so                         Библиотека yuv для конвертации изображений на CPU
src/
 ...                                    Исходный код проекта
third-party/
 ---- cxxopts/                          Библиотека для парсинга параметров из командной строки
 ---- Modbus/                           Стандартная библиотека для испольвания ModBus RTU/TCP

```
## Возможности

Проект использует множество способов оптимизации пайплайна, из за чего достигается высокая производительность:

- Аппаратное декодирование MJPEG, H.264 и др.
- Аппаратное конвертирование форматов изображения NV12, NV16 и других в пригодный для ИИ формат RGB
- Аппаратное ускорение при выводе изображения на экран при помощи RockChip DRM
- Zero-copy передача данных внутри приложения
- Многопоточность

## Сборка и запуск

Можно воспользоваться версией проекта с прекомпилированными библиотеками, тогда сборка происходит так:

```bash
git clone https://github.com/Matveyee/realtime-inference.git
cd realtime-inference
git submodule update --init --recursive
mkdir build
cd build
cmake ..
make
```

Вот пример запуска
```
sudo ./main --input-width 1920  --input-height 1080 --camera-path /dev/video40 --model-path ~/yolov11n.hef --nodetect --frame-rate 30
```
Подробнее в документации doc.txt
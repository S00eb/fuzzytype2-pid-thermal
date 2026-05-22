# ============================================================
# GNUPlot Script - Master Plotting IT2 Fuzzy-PID
# (Hanya 4 Grafik Esensial untuk Tesis)
# ============================================================

set datafile separator ","
set grid

# ============================================================
# 1. Plot Gabungan (Suhu, Cahaya, PWM vs Waktu)
# ============================================================
set terminal pngcairo size 900,1000 enhanced font "Arial,11"
set output "plot_gabungan.png"

set multiplot layout 3,1 title "Dinamika Sistem: Suhu, Cahaya, dan Respons PWM" font ",14"

# Panel 1: Suhu
set xlabel ""
set title "Suhu Inverter" font ",12"
set ylabel "Suhu (°C)" font ",10"
set yrange [20:80]
set xrange [0:50]
plot "data_simulasi.csv" every ::1 using 1:2 with linespoints pt 7 ps 0.5 lw 2 lc rgb "#0066CC" title "Suhu"

# Panel 2: Cahaya
set title "Intensitas Cahaya" font ",12"
set ylabel "Cahaya (Lux)" font ",10"
set yrange [0:1000]
plot "data_simulasi.csv" every ::1 using 1:3 with linespoints pt 7 ps 0.5 lw 2 lc rgb "#FF9900" title "Cahaya"

# Panel 3: PWM
set title "Keluaran Kendali PWM" font ",12"
set xlabel "Waktu (detik)" font ",11"
set ylabel "PWM (0-999)" font ",10"
set yrange [0:1100]
set arrow from 0,350 to 50,350 nohead dt 2 lc rgb "#FF6600" lw 1.5
plot "data_simulasi.csv" every ::1 using 1:4 with linespoints pt 7 ps 0.5 lw 2 lc rgb "#CC3300" title "PWM"
unset arrow

unset multiplot

# ============================================================
# 2. Scatter Plot: Suhu vs PWM
# ============================================================
set terminal pngcairo size 900,450 enhanced font "Arial,11"
set output "plot_scatter_suhu.png"
set title "Korelasi Suhu Inverter terhadap Nilai PWM" font ",13"
set xlabel "Suhu (°C)" font ",11"
set ylabel "PWM (0-999)" font ",11"
set xrange [20:80]
set yrange [0:1100]

plot "data_simulasi.csv" every ::1 using 2:4 with points pt 7 ps 1.0 lc rgb "#800080" title "Titik Data Operasi"

# ============================================================
# 3. Scatter Plot: Cahaya vs PWM
# ============================================================
set output "plot_scatter_cahaya.png"
set title "Korelasi Intensitas Cahaya terhadap Nilai PWM" font ",13"
set xlabel "Cahaya (Lux)" font ",11"
set ylabel "PWM (0-999)" font ",11"
set xrange [0:1000]
set yrange [0:1100]

plot "data_simulasi.csv" every ::1 using 3:4 with points pt 7 ps 1.0 lc rgb "#009933" title "Titik Data Operasi"

# ============================================================
# 4. 3D Plot: Control Surface
# ============================================================
set terminal pngcairo size 900,700 enhanced font "Arial,11"
set output "plot_surface_3d.png"

# Mengatur margin agar teks sebelah kiri tidak terpotong
set lmargin 14
set rmargin 6
set bmargin 5
set tmargin 5

set title "Permukaan Kendali (Control Surface) IT2 Fuzzy-PID" font ",14"
set xlabel "Suhu Aktual (°C)" offset 0,-1 font ",12"
set ylabel "Intensitas Cahaya (Lux)" offset 0,-1 font ",12"
set zlabel "Keluaran PWM (0-999)" offset -1,0 font ",12" rotate by 90

set dgrid3d 50,50 qnorm 2
set hidden3d
set pm3d
set palette defined (0 "blue", 1 "green", 2 "yellow", 3 "red")
set view 50, 45, 1, 1
set xrange [20:80]
set yrange [200:800]
set zrange [0:1100]

splot "data_simulasi.csv" every ::1 using 2:3:4 with pm3d notitle

### 1. Cài đặt: 
requirements.txt, nhẹ thôi ae ạ

### 2. Giải thích

hiện đã vẽ xong bản đồ 2D (anh em xem file 1.PNG và chạy thử grid_wall.py - file này tự đổi ảnh đen trắng thành bản đồ 2D)

giả lập hiện được thiết kế như sau:
- đơn vị đo: 1 pixel trên ảnh = 0.1m (tổng kích thước 147.9m x 61.5m, t đo tương đối thôi chứ thực tế ở delta rộng hơn)
- thời gian: dt = 0.01s (giả lập update 100 lần mỗi giây)
- khi chạy giả lập:
  + Spawning (`floorplan.py`, `simulation.py`): hai file này tự động tìm các phòng trong ảnh 1.png và spawn ngẫu nhiên agents trong các phòng.
  + Logic di chuyển của agent khá lằng nhằng (vibe code thôi)
  + Cửa thoát hiểm: hiện đang thiết lập là nếu có quá 25 người trong bán kính 10m (phải nhìn thấy cửa, agent sau tường không tính vào ngưỡng này) thì sẽ đóng - không ai đi qua được nữa. Và cửa chỉ mở lại nếu đủ 2 điều kiện: ít nhất 10 giây sau khi đóng và không quá 15 người trong bán kính 10m (vẫn phải nhìn thấy cửa).

tất cả các con số đều có thể chỉnh trong config.py

### 3. Chạy giả lập:

mọi lần chạy đều lưu lại kết quả trong root/results/(thời gian), mặc định lưu .json, .csv và .png

python run.py

một số các biến quan trọng như sau:
-- agents: số lượng người trong giả lập (mặc định 200)
-- close-above: ngưỡng quyết định đóng cửa (đang để tạm 25 người thì cửa sẽ đóng hẳn luôn, không đi qua được)
-- reopen-at-most: ngưỡng quyết định mở lại (đang để tạm 15)
-- min-closed: ngưỡng thời gian đóng cửa tối thiểu (đang để tạm 10)
-- t-max: thời gian tối đa chạy giả lập (nên là dưới 60 giây, vì sau 60 giây mà chưa thoát ra ngoài hết thì hệ thống có vấn đề. đang để mặc định 300 giây nhưng nếu agents ra hết thì tự dừng giả lập)
-- save: lưu file .gif hoặc .mp4 (nên đổi tên file mỗi lần chạy vì sợ ghi đè)

ví dụ:

python run.py --agents 300 --save run.mp4 (chạy lần 1 với 300 agents, lưu ra file run.mp4)
python run.py --agents 200 --t-max 60 --save run2.mp4 (chạy lần 2 với 200 agents, tối đa 60 giây, lưu file khác)

### 4. Cảm biến camera (vùng nhìn hình nón, mô phỏng thiết bị thật)

Camera trên biển thoát hiểm là **hình nón nhìn từ cửa ra hành lang**, theo OmniVision OV5648 (HFOV 58.6°, VFOV 45.3°). Tham số trong `SensorConfig` (config.py):

- `radius` 8 m (dùng 8-10 m; trên 10 m chưa nên tin độ chính xác), `fov_deg` 58.6 (360 = vùng tròn cũ)
- Vùng mù dưới camera, tính từ `mount_height` (2.5 m), `tilt_deg` (30°), `vfov_deg`. **Hai số này là giá trị giả định, thay bằng số đo thật.** Tắt bằng `use_min_range = False`.
- `period` 1.0 s: bộ đếm AI (CSRNet / DM-Count) chạy trên thiết bị nhúng không đọc 10 lần mỗi giây
- `count_error` 0.10: AI đếm sai số tương đối ~10%
- `metric` = `density` (người/m² trên diện tích vùng nhìn, ~26 m²) hoặc `count`; `smooth_window` 3 s (trung bình trượt)

### 5. Logic cửa

Mỗi cửa có 2 trạng thái: `OPEN` (xanh) và `CLOSED` (đỏ).

- Đóng (`CLOSED`): giá trị camera > `close_above` (0.7 người/m²) và cửa đã mở ≥ `min_open_time` (5 s).
- Mở lại (`OPEN`): đã đóng ≥ `min_closed_time` (10 s) và giá trị ≤ `reopen_at_most` (0.4 người/m²), hoặc **bắt buộc mở sau `max_closed_time` (20 s)** dù đông (chống kẹt).
- Khi biển ở trạng thái `CLOSED` (hiển thị đỏ / `CLOSED`): cửa vật lý vẫn mở, các agent vẫn đi về phía biển thoát hiểm gần nhất (dựa theo `Crowd`) bất kể trạng thái đóng/mở của biển.

### 6. Kịch bản quá tải

Cửa vẽ trong bản đồ rộng 5.6 m, rộng hơn cửa thoát hiểm thật (1.0-2.4 m) nên ít khi quá tải. Dùng:

python run.py --agents 300 --headless --door-width 1.2 --spawn-x-max 60     (cửa 1.2 m, đám đông dồn nửa trái)
python scenarios.py --seeds 3 --only left_1m2,left_1m0                      (so sánh các cấu hình, ghi results/scenarios.csv)

`summary.json` có thêm `peak_agents_in_danger` và `danger_person_seconds` (người bị ép hơn `danger_force_N` = 1000 N, **ngưỡng này là giả định, chỉnh trong config.py**).

Ví dụ khác:

python run.py --agents 300 --headless --no-gates   (đối chứng)

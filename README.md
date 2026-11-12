### 1. Cài đặt: 
requirements.txt, nhẹ thôi ae ạ

### 2. Giải thích

hiện đã vẽ xong bản đồ 2D (anh em xem file 1.PNG và chạy thử grid_wall.py - file này tự đổi ảnh đen trắng thành bản đồ 2D)

giả lập hiện được thiết kế như sau:
- đơn vị đo: 1 pixel trên ảnh = 0.05m (tổng kích thước 69.1m x 37.5m, đo tương đối)
- thời gian: dt = 0.01s (giả lập update 100 lần mỗi giây)
- khi chạy giả lập: hai file và `floorplan.py`, `simulation.py` tự động tìm các phòng trong ảnh 1.png và spawn ngẫu nhiên agents trong các phòng.

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

### Phase 2 - Dũng:

### 4. Cảm biến camera (vùng nhìn hình nón, mô phỏng thiết bị thật)

Camera trên biển thoát hiểm là **hình nón nhìn từ cửa ra hành lang**, theo OmniVision OV5648 (HFOV 58.6°, VFOV 45.3°). Tham số trong `SensorConfig` (config.py):

| Thông số | Giá trị | Ghi chú |
|---|---|---|
| Góc nhìn ngang | 58,6° | Nón hướng từ cửa ra hành lang |
| Tầm đếm | 8 m | Có thể chỉnh 8 đến 10 m, cần test thực tế |
| Vùng mù dưới camera | tính từ vị trí lắp | **Chiều cao 2,5 m và góc chúc 30° là giá trị giả định, cần thay bằng số đo thật** |
| Chu kỳ đọc | 1 giây | cập nhật model mỗi giây |
| Sai số đếm | ±10% | Mô hình được phép sai số trong khoảng 10%, chỉnh sau khi có kết quả test video cuối cùng |
| Làm mượt | trung bình 3 giây | Kết quả của mô hình được tính dựa vào frame hiện tại + 2 frame trước |

model giờ chỉ đếm được người trong phạm vi nón được thiết lập.

### 5. Logic cửa

Mỗi cửa có 2 trạng thái: `OPEN` (xanh) và `CLOSED` (đỏ).

- Đóng (`CLOSED`): giá trị camera > `close_above` (0.7 người/m²) và cửa đã mở ≥ `min_open_time` (5 s).
- Mở lại (`OPEN`): đã đóng ≥ `min_closed_time` (10 s) và giá trị ≤ `reopen_at_most` (0.4 người/m²), hoặc bắt buộc mở sau `max_closed_time` (20 s)

hiện đã chỉnh lại để cửa không thể đóng mà chỉ bật đèn đỏ

### Phase 3 - Map update - Vinh

Toàn bộ bản đồ được vẽ lại để chính xác hơn, update tầng 2 thư viện

### Phase 4 - logic của người thoát hiểm

Hiện mọi người trong giả lập đều mặc định tìm lối ra gần nhất để chạy, không quan tâm là trước mặt mình bao nhiêu người và các cửa màu xanh hay đỏ (có file run.mp4 t up lên, bạn để ý từ 0:24 - 0:47 là mọi người vẫn bị dồn vào cửa đông trong khi bên kia ko có ai)

Nhờ bạn nghiên cứu một số câu hỏi nhé:

1. Trung bình trên delta trường mình (tầng 2, 3, 4) có bao nhiêu người trong các slot học? 
- không thể lấy số phòng x số người trung bình vì có các lab, các phòng học trống (đến giờ học nhưng không ai học)
- rất khó lấy được con số đúng nhưng càng chính xác càng tốt nhé, cái này quan trọng vì ảnh hưởng trực tiếp đến kết quả giả lập

2. Logic của người thoát hiểm: Khi thấy cửa phía trước mình đông (đèn đỏ) nhưng có cửa khác xa hơn vắng người (đèn xanh) thì mọi người sẽ làm gì?
- video t up lên (run.mp4) chứng minh là nếu mọi người cùng chạy về cửa gần nhất thì sẽ ùn tắc rất nghiêm trọng. Tuy nhiên, không thể nói rằng tất cả mọi người sẽ chạy sang cửa vắng vì lúc thoát hiểm rất loạn, mỗi người sẽ có một cách xử lý khác nhau
- mình cần nghiên cứu và đưa ra một logic tìm đường chung cho mọi người (if 'cửa gần nhất đỏ' and 'có cửa xanh trong tầm nhìn' and 'khoảng cách giữa cửa đỏ và mình lớn hơn bao nhiêu mét' then 'chạy sang cửa vắng' với tỉ lệ bao nhiêu)
- nếu được thì tìm paper, tài liệu nghiên cứu về vấn đề này (khả năng cao là có)

3. Kịch bản thử nghiệm (nếu xong kịp 2 cái trên trước tối t6)
- Khi đã xây xong giả lập chạy ổn và logic người thoát hiểm, cần phải có một kịch bản so sánh phù hợp để chứng minh là hệ thống của mình giúp người thoát nạn nhanh hơn.
- Ví dụ: nếu có 100 người thì không khác biệt, nhưng nếu 400 người thì hệ thống của mình giúp tất cả mọi người ra nhanh hơn 30%.
- nghiên cứu best-case, worst-case, asymmetry load, ...
- Dũng có một file scenarios.py test các kịch bản khá ổn nhưng còn đơn giản - cần nghiên cứu kĩ hơn về các con số cụ thể
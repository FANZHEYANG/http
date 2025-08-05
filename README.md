# 温度部分
>前端发送请求——http服务器接收——iray_pro开始温度检测——发送回服务器——传到前端
## 前端请求温度数据
```javascript
 function getTemperatureData() {
      const statusDiv = document.getElementById('temperature-status');
      const button = document.getElementById('getTempBtn');
      
      statusDiv.textContent = '正在获取温度数据...';
      button.disabled = true;
      button.textContent = '🌡️ 获取中...';
      
      fetch('http://192.168.31.70:8080/robot/temp')
        .then(res => res.json())
        .then(data => {
          const tempInfo = `平均温度: ${data.average_temperature}°C, 中位数温度: ${data.medium_temperature}°C`;
          statusDiv.textContent = `✅ 温度数据获取成功 - ${tempInfo}`;
          
          // 在数据区域显示温度信息
          appendDataLine(`🌡️ [温度数据] ${tempInfo}`, "event-temperature");
        })
        .catch(err => {
          statusDiv.textContent = '❌ 获取温度数据失败: ' + err;
          appendDataLine(`❌ [温度数据] 获取失败: ${err}`, "event-error");
        })
        .finally(() => {
          button.disabled = false;
          button.textContent = '🌡️ 获取温度数据';
        });
    }
```
# 气体部分
>气体传感器 → UDP协议 → 后端服务器 → 数据处理 → SSE推送 → 前端显示
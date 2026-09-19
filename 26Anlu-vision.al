<?xml version="1.0" encoding="utf-8"?>
<Project xmlns:xsd="http://www.w3.org/2001/XMLSchema" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
  <Name>26Anlu-vision</Name>
  <Device>EG4S20BG256</Device>
  <TopModule>top</TopModule>
  <Version>6.2</Version>
  <SourceFiles>
    <!-- Top module -->
    <File>top.v</File>
    
    <!-- Clock & Reset -->
    <File>src/util/por_generator.v</File>
    <File>ip/video_pll.v</File>
    
    <!-- Camera drivers -->
    <File>src/camera/ov5640_dri.v</File>
    <File>src/camera/i2c_dri.v</File>
    <File>src/camera/i2c_ov5640_rgb565_cfg.v</File>
    <File>src/camera/cmos_capture_data.v</File>
    <File>src/camera/ov5640_delay.v</File>
    
    <!-- SDRAM controller -->
    <File>src/sdram/sdram.v</File>
    <File>src/sdram/enc_file/global_def.v</File>
    <File>src/sdram/enc_file/sdr_as_ram.enc.v</File>
    <File>src/sdram/enc_file/sdr_init_ref.enc.v</File>
    <File>src/sdram/enc_file/sdr_wrrd.enc.v</File>
    
    <!-- Frame buffer control -->
    <File>src/memory/frame_read_write.v</File>
    <File>src/memory/frame_fifo_write.v</File>
    <File>src/memory/frame_fifo_read.v</File>
    
    <!-- Video timing & display -->
    <File>src/video/video_timing_data.v</File>
    <File>src/video/color_bar.v</File>
    <File>src/video/video_define.v</File>
    <File>src/video/video_delay.v</File>
    
    <!-- HDMI transmitter (VHDL) -->
    <File>src/hdmi/hdmi_tx.vhd</File>
    <File>src/hdmi/enc_file/DVITransmitter.enc.vhd</File>
    <File>src/hdmi/enc_file/SerializerN_1_lvds.enc.vhd</File>
    <File>src/hdmi/enc_file/SerializerN_1_lvds_dat.enc.vhd</File>
    <File>src/hdmi/enc_file/TMDSEncoder.enc.vhd</File>
    
    <!-- IP cores -->
    <File>ip/afifo_16_32_256.v</File>
    <File>ip/afifo_32_16_256.v</File>
    <File>ip/line_ram_640x8.v</File>
    <File>ip/line_ram_640x8_1.v</File>
  </SourceFiles>
  <Constraints>
    <File>top.adc</File>
    <File>top.sdc</File>
  </Constraints>
  <IPs>
    <IP>ip/video_pll.ipc</IP>
    <IP>ip/afifo_16_32_256.ipc</IP>
    <IP>ip/afifo_32_16_256.ipc</IP>
  </IPs>
</Project>

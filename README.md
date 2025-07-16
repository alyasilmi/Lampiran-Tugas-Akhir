#!/usr/bin/env python3

import rospy
import time
import numpy as np
import json
import os
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import WheelsCmdStamped
from datetime import datetime
import math

class HoughMonitoringIntegrated:
    """
    INTEGRATED MONITORING NODE - Sesuai dengan sistem yang sudah ada
    Subscribe ke output dari EarlyTurnDetectionNode
    Menghitung 6 data penting + integrasi dengan data collection existing
    """
    def __init__(self):
        rospy.init_node('hough_monitoring_integrated', anonymous=True)
        
        # SUBSCRIBE ke output dari EarlyTurnDetectionNode yang sudah ada
        rospy.Subscriber('/lane_detection/steering_angle', Float32, self.steering_callback)
        rospy.Subscriber('/lane_detection/lane_detected', Bool, self.detection_callback)
        rospy.Subscriber('/duckduck/camera_node/image/compressed', CompressedImage, self.frame_callback)
        rospy.Subscriber('/duckduck/wheels_driver_node/wheels_cmd', WheelsCmdStamped, self.wheels_callback)
        
        # Data collection untuk 6 metrics penting
        self.data_collection_active = False
        self.test_start_time = None
        
        # Test conditions - disesuaikan dengan sistem yang sudah ada
        self.test_conditions = {
            "1": "LURUS_TERANG", "2": "LURUS_REDUP", "3": "LURUS_GELAP",
            "4": "KIRI_TERANG", "5": "KIRI_REDUP", "6": "KIRI_GELAP", 
            "7": "KANAN_TERANG", "8": "KANAN_REDUP", "9": "KANAN_GELAP",
            "10": "MELINGKAR_TERANG", "11": "MELINGKAR_REDUP", "12": "MELINGKAR_GELAP",
            "13": "KESELURUHAN_TERANG", "14": "KESELURUHAN_REDUP", "15": "KESELURUHAN_GELAP",
            # Tambahan untuk jurnal
            "16": "POSITION_0CM_ANGLE_0DEG", "17": "POSITION_0CM_ANGLE_20DEG",
            "18": "POSITION_24CM_ANGLE_0DEG", "19": "POSITION_24CM_ANGLE_10DEG", 
            "20": "POSITION_24CM_ANGLE_NEG10DEG", "21": "POSITION_30CM_ANGLE_0DEG"
        }
        
        self.current_test_condition = "LURUS_TERANG"
        
        # 1. Detection Rate calculation
        self.total_detection_attempts = 0    # Total frame yang diproses
        self.successful_detections = 0       # Frame yang berhasil detect jalur
        
        # 2. Accuracy calculation (Standard formula)
        self.true_positives = 0      # Detect jalur yang memang ada
        self.false_positives = 0     # Detect jalur yang tidak ada (noise)
        self.false_negatives = 0     # Gagal detect jalur yang ada
        self.true_negatives = 0      # Benar tidak detect (tidak ada jalur)
        
        # 3. Position Error calculation
        self.position_errors = []    # Lateral position error dalam cm
        self.angle_errors = []       # Angle error dalam degrees
        self.lane_center_reference = 320  # Center image reference (640px width)
        
        # 4. Processing Time calculation
        self.processing_times = []   # Processing time per frame
        self.frame_start_time = None
        self.detection_start_time = None
        
        # 5. Total Frames counter
        self.total_frames = 0
        
        # Ground truth management
        self.ground_truth_lanes_exist = True  # Controlled environment assumption
        
        # Success Rate tracking (kompatibel dengan sistem existing)
        self.failed_detections = 0
        
        # Error rate tracking
        self.steering_errors = []
        self.expected_steering = 0.0  # Set based on test condition
        
        # Results storage - menggunakan folder yang sama dengan sistem existing
        self.results_folder = "/data/hough_monitoring_results"
        self.setup_results_folder()
        
        # Real-time snapshots untuk detailed analysis
        self.realtime_snapshots = []
        self.metrics_snapshots = []  # Kompatibel dengan sistem existing
        
        # Test data storage - format yang sama dengan sistem existing
        self.test_data = {}  # Store multiple test results
        
        # Real-time display - interval yang sama dengan sistem existing
        self.display_timer = rospy.Timer(rospy.Duration(2.0), self.display_metrics)
        
        rospy.loginfo("=== HOUGH MONITORING INTEGRATED STARTED ===")
        rospy.loginfo("Compatible dengan data collection system yang sudah ada")
        rospy.loginfo("Menghitung 6 data penting + 4 data existing system")
        
        self.interactive_menu()
    
    def interactive_menu(self):
        """Interactive menu - format yang sama dengan sistem existing"""
        while not rospy.is_shutdown():
            try:
                print("\n" + "="*70)
                print("🔬 HOUGH MONITORING INTEGRATED - 6+4 DATA LENGKAP")
                print("="*70)
                print("Current condition:", self.current_test_condition)
                print("Collection active:", "✅ YES" if self.data_collection_active else "❌ NO")
                print("\nTest Conditions:")
                print("  1-3:   Lurus (Terang/Redup/Gelap)")
                print("  4-6:   Belok Kiri (Terang/Redup/Gelap)")  
                print("  7-9:   Belok Kanan (Terang/Redup/Gelap)")
                print("  10-12: Melingkar (Terang/Redup/Gelap)")
                print("  13-15: Keseluruhan (Terang/Redup/Gelap)")
                print("  16-21: Position Tests (Journal conditions)")
                print("\nCommands:")
                print("  s:     Start/Stop collection")
                print("  r:     Show results")
                print("  save:  Save results to file")
                print("  rt:    Save real-time data")
                print("  disp:  Save display snapshots")  # TAMBAH COMMAND BARU
                print("  clear: Clear current data")
                print("  q:     Quit")
                
                command = input("\nEnter command: ").strip().lower()
                
                if command in self.test_conditions:
                    self.set_test_condition(self.test_conditions[command])
                elif command == 's':
                    self.toggle_monitoring()
                elif command == 'r':
                    self.show_detailed_results()
                elif command == 'save':
                    self.save_results_to_file()
                elif command == 'rt':
                    self.save_realtime_data()
                elif command == 'disp':
                    self.save_display_snapshots()  # TAMBAH COMMAND HANDLER
                elif command == 'clear':
                    self.clear_current_data()
                elif command == 'q':
                    break
                else:
                    print("❌ Invalid command!")
                    
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"Error: {e}")
        
        rospy.loginfo("Hough monitoring stopped")
    
    def set_test_condition(self, condition):
        """Set test condition dan expected behavior - sama seperti sistem existing"""
        self.current_test_condition = condition
        
        # Set expected steering berdasarkan kondisi (sama seperti sistem existing)
        if "LURUS" in condition or "POSITION_0CM_ANGLE_0DEG" in condition:
            self.expected_steering = 0.0
        elif "KIRI" in condition or "NEG10DEG" in condition:
            self.expected_steering = -0.5  # Expected left steering
        elif "KANAN" in condition or "ANGLE_20DEG" in condition or "ANGLE_10DEG" in condition:
            self.expected_steering = 0.5   # Expected right steering
        elif "MELINGKAR" in condition:
            self.expected_steering = 0.3   # Variable steering
        else:  # KESELURUHAN or other position tests
            self.expected_steering = 0.0   # Variable
        
        rospy.loginfo(f"Test condition set to: {condition}")
        rospy.loginfo(f"Expected steering: {self.expected_steering}")
    
    def toggle_monitoring(self):
        """Start/Stop monitoring - sama seperti sistem existing"""
        if not self.data_collection_active:
            self.data_collection_active = True
            self.test_start_time = time.time()
            self.clear_current_data()
            rospy.loginfo(f"✅ Hough monitoring STARTED for: {self.current_test_condition}")
        else:
            self.data_collection_active = False
            rospy.loginfo(f"⏹️  Hough monitoring STOPPED for: {self.current_test_condition}")
            self.calculate_final_metrics()
    
    def clear_current_data(self):
        """Clear current session data - sama seperti sistem existing"""
        self.total_frames = 0
        self.total_detection_attempts = 0
        self.successful_detections = 0
        self.failed_detections = 0
        
        self.true_positives = 0
        self.false_positives = 0
        self.false_negatives = 0
        self.true_negatives = 0
        
        self.position_errors = []
        self.angle_errors = []
        self.processing_times = []
        self.steering_errors = []
        self.realtime_snapshots = []
        self.metrics_snapshots = []
        
        # CLEAR display snapshots
        self.display_snapshots = []
        
        rospy.loginfo("🗑️  Hough monitoring data cleared")
    
    def setup_results_folder(self):
        """Setup folder - kompatibel dengan sistem existing"""
        if not os.path.exists(self.results_folder):
            try:
                os.makedirs(self.results_folder)
                rospy.loginfo(f"Created hough results folder: {self.results_folder}")
            except PermissionError:
                self.results_folder = "/tmp/hough_monitoring_results"
                os.makedirs(self.results_folder, exist_ok=True)
                rospy.logwarn(f"Using fallback folder: {self.results_folder}")
    
    def frame_callback(self, msg):
        """Track frame timing dan total frames"""
        if not self.data_collection_active:
            return
        
        current_time = time.time()
        
        # Start timing untuk processing time calculation
        if self.frame_start_time is not None:
            processing_time = current_time - self.frame_start_time
            self.processing_times.append(processing_time)
        
        self.frame_start_time = current_time
        self.total_frames += 1
    
    def detection_callback(self, msg):
        """Process detection results untuk accuracy calculation"""
        if not self.data_collection_active:
            return
        
        self.detection_start_time = time.time()
        
        lane_detected = msg.data
        self.total_detection_attempts += 1
        
        # Ground truth determination
        lane_should_exist = self.determine_ground_truth()
        
        # Update confusion matrix
        if lane_should_exist:
            if lane_detected:
                quality_score = self.assess_detection_quality()
                if quality_score > 0.7:  # Good quality detection
                    self.true_positives += 1
                    self.successful_detections += 1
                else:
                    self.false_positives += 1
            else:
                self.false_negatives += 1
                self.failed_detections += 1
        else:
            if lane_detected:
                self.false_positives += 1
            else:
                self.true_negatives += 1
        
        # Store snapshot for real-time display
        self.store_realtime_snapshot()
    
    def steering_callback(self, msg):
        """Process steering angle untuk position & angle error calculation"""
        if not self.data_collection_active:
            return
        
        steering_angle = msg.data  # Range: -1.0 to 1.0
        
        # Calculate position error (lateral deviation)
        max_deviation_cm = 10.0  # Maksimal 10 cm deviation (sesuai jurnal)
        estimated_position_error = abs(steering_angle) * max_deviation_cm
        self.position_errors.append(estimated_position_error)
        
        # Calculate angle error (orientation error)
        max_angle_error_deg = 5.0  # Maksimal 5 degree error (sesuai jurnal)
        estimated_angle_error = abs(steering_angle) * max_angle_error_deg
        self.angle_errors.append(estimated_angle_error)
        
        # Calculate steering error (untuk sistem existing)
        if "KESELURUHAN" not in self.current_test_condition and "MELINGKAR" not in self.current_test_condition:
            error = abs(steering_angle - self.expected_steering)
            self.steering_errors.append(error)
        else:
            error = abs(steering_angle)
            self.steering_errors.append(error)
    
    def wheels_callback(self, msg):
        """Process wheel commands - kompatibel dengan sistem existing"""
        if not self.data_collection_active:
            return
        # Additional wheel data processing if needed
    
    def determine_ground_truth(self):
        """Determine apakah jalur SEHARUSNYA ada"""
        # Dalam controlled environment, assume lanes always exist
        # kecuali ada indikasi robot off-track
        
        recent_detections = 10
        if len(self.position_errors) >= recent_detections:
            recent_errors = self.position_errors[-recent_detections:]
            avg_recent_error = np.mean(recent_errors)
            if avg_recent_error > 8.0:  # Sangat tinggi error
                return False  # Possibly off-track
        
        return True  # Default: lanes should exist
    
    def assess_detection_quality(self):
        """Assess kualitas detection untuk filter false positives"""
        quality_score = 1.0
        
        # Factor 1: Consistency check
        if len(self.position_errors) >= 3:
            recent_errors = self.position_errors[-3:]
            error_variance = np.var(recent_errors)
            if error_variance > 2.0:  # High variance = poor quality
                quality_score *= 0.6
        
        # Factor 2: Processing time check
        if len(self.processing_times) >= 3:
            recent_times = self.processing_times[-3:]
            avg_time = np.mean(recent_times)
            if avg_time > 0.1:  # Slow processing = struggling
                quality_score *= 0.7
        
        return quality_score
    
    def calculate_hough_metrics(self):
        """Calculate 6 data penting sesuai jurnal"""
        if self.total_frames == 0:
            return None
        
        # 1. DETECTION RATE (Journal Table 1 format)
        total_expected = self.true_positives + self.false_negatives
        detection_rate = (self.true_positives / total_expected * 100) if total_expected > 0 else 0
        
        # 2. ACCURACY (Standard formula)
        total_predictions = (self.true_positives + self.true_negatives + 
                           self.false_positives + self.false_negatives)
        accuracy = ((self.true_positives + self.true_negatives) / total_predictions * 100) if total_predictions > 0 else 0
        
        # 3. POSITION ERROR (Journal Table 2 format)
        position_error = np.mean(self.position_errors) if self.position_errors else 0.0
        
        # 4. ANGLE ERROR (Journal Table 2 format)
        angle_error = np.mean(self.angle_errors) if self.angle_errors else 0.0
        
        # 5. PROCESSING TIME (Journal Table 3 format)
        processing_time = np.mean(self.processing_times) if self.processing_times else 0.0
        
        # 6. TP/FP/FN/TN counts
        tp_fp_fn_tn = f"{self.true_positives}/{self.false_positives}/{self.false_negatives}/{self.true_negatives}"
        
        return {
            'detection_rate': detection_rate,
            'accuracy': accuracy,
            'position_error_cm': position_error,
            'angle_error_deg': angle_error,
            'processing_time_s': processing_time,
            'tp_fp_fn_tn': tp_fp_fn_tn,
            'total_frames': self.total_frames
        }
    
    def calculate_existing_system_metrics(self):
        """Calculate 4 data dari sistem existing"""
        if self.total_frames == 0:
            return None
        
        total_detections = self.successful_detections + self.failed_detections
        
        # 1. Akurasi (Detection Accuracy) - berbeda dari accuracy di atas
        existing_accuracy = (self.successful_detections / total_detections * 100) if total_detections > 0 else 0
        
        # 2. Waktu Proses (sama seperti processing time)
        avg_processing_time_ms = np.mean(self.processing_times) * 1000 if self.processing_times else 0
        
        # 3. Tingkat Error (Steering Error)
        avg_steering_error = np.mean(self.steering_errors) if self.steering_errors else 0
        
        # 4. Success Rate
        success_rate = (self.successful_detections / self.total_frames * 100) if self.total_frames > 0 else 0
        
        return {
            'akurasi_persen': existing_accuracy,
            'waktu_proses_ms': avg_processing_time_ms,
            'tingkat_error': avg_steering_error,
            'success_rate_persen': success_rate
        }
    
    def store_realtime_snapshot(self):
        """Store real-time snapshot - kompatibel dengan kedua sistem"""
        if not self.data_collection_active:
            return
        
        current_time = time.time()
        relative_time = current_time - self.test_start_time if self.test_start_time else 0
        
        hough_metrics = self.calculate_hough_metrics()
        existing_metrics = self.calculate_existing_system_metrics()
        
        if hough_metrics and existing_metrics:
            # Snapshot untuk Hough monitoring
            hough_snapshot = {
                'timestamp': current_time,
                'relative_time_seconds': relative_time,
                'condition': self.current_test_condition,
                'detection_rate': hough_metrics['detection_rate'],
                'accuracy': hough_metrics['accuracy'],
                'position_error_cm': hough_metrics['position_error_cm'],
                'angle_error_deg': hough_metrics['angle_error_deg'],
                'processing_time_s': hough_metrics['processing_time_s'],
                'total_frames': hough_metrics['total_frames'],
                'tp': self.true_positives,
                'fp': self.false_positives,
                'fn': self.false_negatives,
                'tn': self.true_negatives
            }
            self.realtime_snapshots.append(hough_snapshot)
            
            # Snapshot untuk sistem existing
            existing_snapshot = {
                'timestamp': current_time,
                'relative_time_seconds': relative_time,
                'condition': self.current_test_condition,
                'akurasi_persen': round(existing_metrics['akurasi_persen'], 1),
                'waktu_proses_ms': round(existing_metrics['waktu_proses_ms'], 1),
                'tingkat_error': round(existing_metrics['tingkat_error'], 3),
                'success_rate_persen': round(existing_metrics['success_rate_persen'], 1),
                'total_frames': self.total_frames,
                'successful_detections': self.successful_detections,
                'failed_detections': self.failed_detections,
                'total_detections': self.successful_detections + self.failed_detections
            }
            self.metrics_snapshots.append(existing_snapshot)
    
    def display_metrics(self, event):
        """Display real-time metrics - gabungan kedua sistem + SIMPAN SNAPSHOT"""
        if not self.data_collection_active or self.total_frames == 0:
            return
        
        hough_metrics = self.calculate_hough_metrics()
        existing_metrics = self.calculate_existing_system_metrics()
        
        if not hough_metrics or not existing_metrics:
            return
        
        # SIMPAN SNAPSHOT INI (yang akan tampil di layar)
        current_time = time.time()
        relative_time = current_time - self.test_start_time if self.test_start_time else 0
        
        display_snapshot = {
            'timestamp': current_time,
            'relative_time_seconds': relative_time,
            'condition': self.current_test_condition,
            # Jurnal metrics
            'detection_rate_display': round(hough_metrics['detection_rate'], 1),
            'accuracy_display': round(hough_metrics['accuracy'], 1),
            'position_error_display': round(hough_metrics['position_error_cm'], 4),
            'angle_error_display': round(hough_metrics['angle_error_deg'], 6),
            'processing_time_display': round(hough_metrics['processing_time_s'], 6),
            'tp_fp_fn_tn_display': hough_metrics['tp_fp_fn_tn'],
            # Existing metrics
            'akurasi_display': round(existing_metrics['akurasi_persen'], 1),
            'waktu_proses_display': round(existing_metrics['waktu_proses_ms'], 1),
            'tingkat_error_display': round(existing_metrics['tingkat_error'], 3),
            'success_rate_display': round(existing_metrics['success_rate_persen'], 1),
            'total_frames_display': self.total_frames,
            'detections_display': f"{self.successful_detections}/{self.successful_detections + self.failed_detections}",
            # Raw values untuk analysis
            'tp': self.true_positives,
            'fp': self.false_positives,
            'fn': self.false_negatives,
            'tn': self.true_negatives,
            'successful_detections': self.successful_detections,
            'failed_detections': self.failed_detections
        }
        
        # TAMBAHKAN ke display snapshots
        if not hasattr(self, 'display_snapshots'):
            self.display_snapshots = []
        self.display_snapshots.append(display_snapshot)
        
        # Display format yang sama dengan sistem existing PLUS 6 data jurnal
        print(f"\n📊 INTEGRATED METRICS - {self.current_test_condition}")
        print("="*70)
        print("🔬 JURNAL METRICS (6 Data Penting):")
        print(f"  🎯 Detection Rate:     {hough_metrics['detection_rate']:.1f}% (Journal Table 1)")
        print(f"  🎯 Accuracy:           {hough_metrics['accuracy']:.1f}%")
        print(f"  📐 Position Error:     {hough_metrics['position_error_cm']:.4f} cm (Journal Table 2)")
        print(f"  📐 Angle Error:        {hough_metrics['angle_error_deg']:.6f}° (Journal Table 2)")
        print(f"  ⏱️  Processing Time:    {hough_metrics['processing_time_s']:.6f} s (Journal Table 3)")
        print(f"  📋 TP/FP/FN/TN:        {hough_metrics['tp_fp_fn_tn']}")
        print("📊 EXISTING SYSTEM METRICS (4 Data):")
        print(f"  🎯 Akurasi:            {existing_metrics['akurasi_persen']:.1f}%")
        print(f"  ⏱️  Waktu Proses:       {existing_metrics['waktu_proses_ms']:.1f} ms")
        print(f"  ❌ Tingkat Error:      {existing_metrics['tingkat_error']:.3f}")
        print(f"  ✅ Success Rate:       {existing_metrics['success_rate_persen']:.1f}%")
        print(f"  📋 Total Frames:       {self.total_frames}")
        print(f"  🔍 Detections:         {self.successful_detections}/{self.successful_detections + self.failed_detections}")
    
    def calculate_final_metrics(self):
        """Calculate final metrics untuk current test"""
        if self.total_frames == 0:
            rospy.logwarn("No data collected!")
            return
        
        hough_metrics = self.calculate_hough_metrics()
        existing_metrics = self.calculate_existing_system_metrics()
        
        if not hough_metrics or not existing_metrics:
            return
        
        test_duration = time.time() - self.test_start_time if self.test_start_time else 0
        
        # Store hasil dalam format yang kompatibel dengan kedua sistem
        self.test_data[self.current_test_condition] = {
            # 6 Data Jurnal
            'detection_rate_percent': round(hough_metrics['detection_rate'], 2),
            'accuracy_percent': round(hough_metrics['accuracy'], 2),
            'position_error_cm': round(hough_metrics['position_error_cm'], 4),
            'angle_error_deg': round(hough_metrics['angle_error_deg'], 6),
            'processing_time_s': round(hough_metrics['processing_time_s'], 6),
            'tp_fp_fn_tn': hough_metrics['tp_fp_fn_tn'],
            
            # 4 Data Sistem Existing  
            'akurasi_persen': round(existing_metrics['akurasi_persen'], 2),
            'waktu_proses_ms': round(existing_metrics['waktu_proses_ms'], 2),
            'tingkat_error': round(existing_metrics['tingkat_error'], 3),
            'success_rate_persen': round(existing_metrics['success_rate_persen'], 2),
            
            # Additional data
            'true_positives': self.true_positives,
            'false_positives': self.false_positives,
            'false_negatives': self.false_negatives,
            'true_negatives': self.true_negatives,
            'total_frames': self.total_frames,
            'successful_detections': self.successful_detections,
            'failed_detections': self.failed_detections,
            'test_duration_seconds': round(test_duration, 1),
            'timestamp': datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        }
        
        rospy.loginfo(f"✅ Integrated metrics calculated for: {self.current_test_condition}")
    
    def show_detailed_results(self):
        """Show detailed results untuk semua test conditions"""
        print("\n" + "="*90)
        print("📈 DETAILED INTEGRATED RESULTS - SEMUA KONDISI PENGUJIAN")
        print("="*90)
        
        if not self.test_data:
            print("❌ No test data available! Run some tests first.")
            return
        
        # Header
        print(f"{'Kondisi':<25} {'DetRate%':<10} {'Acc%':<8} {'PosErr(cm)':<12} {'AngErr(°)':<12} {'ProcTime(s)':<12} {'Akur%':<8} {'SuccRate%':<10}")
        print("-" * 90)
        
        # Data untuk setiap kondisi
        for condition, data in self.test_data.items():
            print(f"{condition:<25} {data['detection_rate_percent']:<10} {data['accuracy_percent']:<8} {data['position_error_cm']:<12} {data['angle_error_deg']:<12} {data['processing_time_s']:<12} {data['akurasi_persen']:<8} {data['success_rate_persen']:<10}")
        
        print("-" * 90)
        
        # Summary statistics
        if len(self.test_data) > 1:
            all_detection_rates = [data['detection_rate_percent'] for data in self.test_data.values()]
            all_accuracies = [data['accuracy_percent'] for data in self.test_data.values()]
            all_position_errors = [data['position_error_cm'] for data in self.test_data.values()]
            all_angle_errors = [data['angle_error_deg'] for data in self.test_data.values()]
            all_processing_times = [data['processing_time_s'] for data in self.test_data.values()]
            all_success_rates = [data['success_rate_persen'] for data in self.test_data.values()]
            
            print("📊 SUMMARY STATISTICS:")
            print(f"Average Detection Rate: {np.mean(all_detection_rates):.2f}%")
            print(f"Average Accuracy:       {np.mean(all_accuracies):.2f}%")
            print(f"Average Position Error: {np.mean(all_position_errors):.4f} cm")
            print(f"Average Angle Error:    {np.mean(all_angle_errors):.6f}°")
            print(f"Average Processing Time:{np.mean(all_processing_times):.6f} s")
            print(f"Average Success Rate:   {np.mean(all_success_rates):.2f}%")
    
    def save_results_to_file(self):
        """Save results dengan format yang kompatibel dengan kedua sistem"""
        if not self.test_data:
            print("❌ No data to save!")
            return
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Save as JSON (comprehensive data)
        json_filename = f"{self.results_folder}/integrated_results_{timestamp}.json"
        with open(json_filename, 'w') as f:
            json.dump(self.test_data, f, indent=2)
        
        # Save as CSV (for analysis/Excel)
        csv_filename = f"{self.results_folder}/integrated_results_{timestamp}.csv"
        with open(csv_filename, 'w') as f:
            f.write("Kondisi,Detection_Rate_%,Accuracy_%,Position_Error_cm,Angle_Error_deg,Processing_Time_s,Akurasi_%,Waktu_Proses_ms,Tingkat_Error,Success_Rate_%,TP,FP,FN,TN,Total_Frames,Duration_s,Timestamp\n")
            
            for condition, data in self.test_data.items():
                f.write(f"{condition},{data['detection_rate_percent']},{data['accuracy_percent']},{data['position_error_cm']},{data['angle_error_deg']},{data['processing_time_s']},{data['akurasi_persen']},{data['waktu_proses_ms']},{data['tingkat_error']},{data['success_rate_persen']},{data['true_positives']},{data['false_positives']},{data['false_negatives']},{data['true_negatives']},{data['total_frames']},{data['test_duration_seconds']},{data['timestamp']}\n")
        
        # Create integrated analysis report
        self.create_integrated_analysis_report(timestamp)
        
        print(f"✅ Integrated results saved:")
        print(f"   📄 JSON: {json_filename}")
        print(f"   📊 CSV:  {csv_filename}")
        print(f"   📋 Report: {self.results_folder}/integrated_analysis_{timestamp}.txt")
    
    def save_realtime_data(self):
        """Save real-time data dengan format yang kompatibel"""
        if not self.realtime_snapshots and not self.metrics_snapshots:
            print("❌ No real-time data to save!")
            return
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Save Hough real-time data
        if self.realtime_snapshots:
            hough_realtime_filename = f"{self.results_folder}/hough_realtime_{timestamp}.csv"
            with open(hough_realtime_filename, 'w') as f:
                f.write("Timestamp,Relative_Time_s,Condition,Detection_Rate_%,Accuracy_%,Position_Error_cm,Angle_Error_deg,Processing_Time_s,Total_Frames,TP,FP,FN,TN\n")
                
                for snap in self.realtime_snapshots:
                    f.write(f"{snap['timestamp']},{snap['relative_time_seconds']:.1f},{snap['condition']},{snap['detection_rate']:.2f},{snap['accuracy']:.2f},{snap['position_error_cm']:.4f},{snap['angle_error_deg']:.6f},{snap['processing_time_s']:.6f},{snap['total_frames']},{snap['tp']},{snap['fp']},{snap['fn']},{snap['tn']}\n")
        
        # Save existing system real-time data
        if self.metrics_snapshots:
            existing_realtime_filename = f"{self.results_folder}/existing_realtime_{timestamp}.csv"
            with open(existing_realtime_filename, 'w') as f:
                f.write("Timestamp,Relative_Time_s,Condition,Akurasi_%,Waktu_Proses_ms,Tingkat_Error,Success_Rate_%,Total_Frames,Successful_Detections,Failed_Detections,Total_Detections\n")
                
                for snap in self.metrics_snapshots:
                    f.write(f"{snap['timestamp']},{snap['relative_time_seconds']:.1f},{snap['condition']},{snap['akurasi_persen']},{snap['waktu_proses_ms']},{snap['tingkat_error']},{snap['success_rate_persen']},{snap['total_frames']},{snap['successful_detections']},{snap['failed_detections']},{snap['total_detections']}\n")
        
        print(f"✅ Real-time data saved:")
        if self.realtime_snapshots:
            print(f"   📈 Hough Real-time: {hough_realtime_filename}")
        if self.metrics_snapshots:
            print(f"   📊 Existing Real-time: {existing_realtime_filename}")
    
    def save_display_snapshots(self):
        """TAMBAHAN: Save snapshots yang tampil di layar (display output)"""
        if not hasattr(self, 'display_snapshots') or not self.display_snapshots:
            print("❌ No display snapshots to save!")
            return
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Save display snapshots as JSON
        display_json_filename = f"{self.results_folder}/display_snapshots_{timestamp}.json"
        display_export = {
            'metadata': {
                'description': 'Display snapshots (yang tampil di layar setiap 2 detik)',
                'total_snapshots': len(self.display_snapshots),
                'export_timestamp': datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                'snapshot_interval': '2 seconds',
                'conditions_included': list(set([snap['condition'] for snap in self.display_snapshots]))
            },
            'display_snapshots': self.display_snapshots
        }
        
        with open(display_json_filename, 'w') as f:
            json.dump(display_export, f, indent=2)
        
        # Save display snapshots as CSV (EXACT format seperti yang tampil di layar)
        display_csv_filename = f"{self.results_folder}/display_snapshots_{timestamp}.csv"
        with open(display_csv_filename, 'w') as f:
            f.write("Timestamp,Relative_Time_s,Condition,Detection_Rate_%,Accuracy_%,Position_Error_cm,Angle_Error_deg,Processing_Time_s,TP_FP_FN_TN,Akurasi_%,Waktu_Proses_ms,Tingkat_Error,Success_Rate_%,Total_Frames,Detections,TP,FP,FN,TN\n")
            
            for snap in self.display_snapshots:
                f.write(f"{snap['timestamp']},{snap['relative_time_seconds']:.1f},{snap['condition']},{snap['detection_rate_display']},{snap['accuracy_display']},{snap['position_error_display']},{snap['angle_error_display']},{snap['processing_time_display']},{snap['tp_fp_fn_tn_display']},{snap['akurasi_display']},{snap['waktu_proses_display']},{snap['tingkat_error_display']},{snap['success_rate_display']},{snap['total_frames_display']},{snap['detections_display']},{snap['tp']},{snap['fp']},{snap['fn']},{snap['tn']}\n")
        
        # Create formatted display report (EXACT seperti output layar)
        display_report_filename = f"{self.results_folder}/display_report_{timestamp}.txt"
        with open(display_report_filename, 'w') as f:
            f.write("="*80 + "\n")
            f.write("DISPLAY SNAPSHOTS REPORT - EXACT OUTPUT DARI LAYAR\n")
            f.write("="*80 + "\n")
            f.write(f"Export Timestamp: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"Total Snapshots: {len(self.display_snapshots)}\n")
            f.write("Format: EXACT seperti yang tampil di layar setiap 2 detik\n\n")
            
            # Group by condition
            conditions_snapshots = {}
            for snap in self.display_snapshots:
                condition = snap['condition']
                if condition not in conditions_snapshots:
                    conditions_snapshots[condition] = []
                conditions_snapshots[condition].append(snap)
            
            for condition, snapshots in conditions_snapshots.items():
                f.write(f"\n{condition} - {len(snapshots)} snapshots:\n")
                f.write("="*70 + "\n")
                
                for i, snap in enumerate(snapshots):
                    f.write(f"\nSnapshot {i+1} (Time: {snap['relative_time_seconds']:.1f}s):\n")
                    f.write("📊 INTEGRATED METRICS - " + condition + "\n")
                    f.write("="*70 + "\n")
                    f.write("🔬 JURNAL METRICS (6 Data Penting):\n")
                    f.write(f"  🎯 Detection Rate:     {snap['detection_rate_display']}% (Journal Table 1)\n")
                    f.write(f"  🎯 Accuracy:           {snap['accuracy_display']}%\n")
                    f.write(f"  📐 Position Error:     {snap['position_error_display']} cm (Journal Table 2)\n")
                    f.write(f"  📐 Angle Error:        {snap['angle_error_display']}° (Journal Table 2)\n")
                    f.write(f"  ⏱️  Processing Time:    {snap['processing_time_display']} s (Journal Table 3)\n")
                    f.write(f"  📋 TP/FP/FN/TN:        {snap['tp_fp_fn_tn_display']}\n")
                    f.write("📊 EXISTING SYSTEM METRICS (4 Data):\n")
                    f.write(f"  🎯 Akurasi:            {snap['akurasi_display']}%\n")
                    f.write(f"  ⏱️  Waktu Proses:       {snap['waktu_proses_display']} ms\n")
                    f.write(f"  ❌ Tingkat Error:      {snap['tingkat_error_display']}\n")
                    f.write(f"  ✅ Success Rate:       {snap['success_rate_display']}%\n")
                    f.write(f"  📋 Total Frames:       {snap['total_frames_display']}\n")
                    f.write(f"  🔍 Detections:         {snap['detections_display']}\n")
                    f.write("\n")
                
                # Summary untuk condition ini
                final_snap = snapshots[-1]
                f.write(f"FINAL METRICS untuk {condition}:\n")
                f.write(f"  Final Detection Rate:   {final_snap['detection_rate_display']}%\n")
                f.write(f"  Final Accuracy:         {final_snap['accuracy_display']}%\n")
                f.write(f"  Final Position Error:   {final_snap['position_error_display']} cm\n")
                f.write(f"  Final Angle Error:      {final_snap['angle_error_display']}°\n")
                f.write(f"  Final Akurasi:          {final_snap['akurasi_display']}%\n")
                f.write(f"  Final Success Rate:     {final_snap['success_rate_display']}%\n")
                f.write(f"  Total Duration:         {final_snap['relative_time_seconds']:.1f} seconds\n")
                f.write(f"  Total Frames:           {final_snap['total_frames_display']}\n\n")
        
        print(f"✅ Display snapshots saved:")
        print(f"   📄 JSON: {display_json_filename}")
        print(f"   📊 CSV:  {display_csv_filename}")
        print(f"   📋 Report: {display_report_filename}")
        print(f"   📝 Total snapshots: {len(self.display_snapshots)}")
    
    def create_integrated_analysis_report(self, timestamp):
        """Create comprehensive analysis report gabungan kedua sistem"""
        report_filename = f"{self.results_folder}/integrated_analysis_{timestamp}.txt"
        
        with open(report_filename, 'w') as f:
            f.write("="*100 + "\n")
            f.write("LAPORAN ANALISIS TERINTEGRASI - HOUGH TRANSFORM & EXISTING SYSTEM\n")
            f.write("HASIL MONITORING LENGKAP UNTUK BAB 4 TUGAS AKHIR\n")
            f.write("="*100 + "\n")
            f.write(f"Timestamp: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"Total Kondisi Diuji: {len(self.test_data)}\n\n")
            
            f.write("METODOLOGI PENGUKURAN:\n")
            f.write("-" * 50 + "\n")
            f.write("🔬 JURNAL METRICS (6 Data Penting):\n")
            f.write("• Detection Rate: TP / (TP + FN) × 100% (Journal Table 1)\n")
            f.write("• Accuracy: (TP + TN) / (TP + TN + FP + FN) × 100%\n")
            f.write("• Position Error: Mean absolute lateral position error (cm)\n")
            f.write("• Angle Error: Mean absolute orientation angle error (degrees)\n")
            f.write("• Processing Time: Mean frame processing time (seconds)\n")
            f.write("• TP/FP/FN/TN: True/False Positives/Negatives counts\n\n")
            
            f.write("📊 EXISTING SYSTEM METRICS (4 Data):\n")
            f.write("• Akurasi: Successful detections / Total detections × 100%\n")
            f.write("• Waktu Proses: Average processing time per frame (ms)\n")
            f.write("• Tingkat Error: Mean steering error from expected value\n")
            f.write("• Success Rate: Successful detections / Total frames × 100%\n\n")
            
            f.write("HASIL PENGUJIAN TERINTEGRASI:\n")
            f.write("-" * 50 + "\n")
            
            for condition, data in self.test_data.items():
                f.write(f"\n{condition}:\n")
                f.write("  🔬 JURNAL METRICS:\n")
                f.write(f"    Detection Rate:    {data['detection_rate_percent']}%\n")
                f.write(f"    Accuracy:          {data['accuracy_percent']}%\n")
                f.write(f"    Position Error:    {data['position_error_cm']} cm\n")
                f.write(f"    Angle Error:       {data['angle_error_deg']}°\n")
                f.write(f"    Processing Time:   {data['processing_time_s']} s\n")
                f.write(f"    Confusion Matrix:  {data['tp_fp_fn_tn']}\n")
                f.write("  📊 EXISTING METRICS:\n")
                f.write(f"    Akurasi:           {data['akurasi_persen']}%\n")
                f.write(f"    Waktu Proses:      {data['waktu_proses_ms']} ms\n")
                f.write(f"    Tingkat Error:     {data['tingkat_error']}\n")
                f.write(f"    Success Rate:      {data['success_rate_persen']}%\n")
                f.write("  📋 ADDITIONAL INFO:\n")
                f.write(f"    Total Frames:      {data['total_frames']}\n")
                f.write(f"    Test Duration:     {data['test_duration_seconds']} s\n")
                f.write(f"    Timestamp:         {data['timestamp']}\n")
            
            # Summary statistics
            if len(self.test_data) > 1:
                # Jurnal metrics averages
                all_detection_rates = [data['detection_rate_percent'] for data in self.test_data.values()]
                all_accuracies = [data['accuracy_percent'] for data in self.test_data.values()]
                all_position_errors = [data['position_error_cm'] for data in self.test_data.values()]
                all_angle_errors = [data['angle_error_deg'] for data in self.test_data.values()]
                all_processing_times = [data['processing_time_s'] for data in self.test_data.values()]
                
                # Existing system averages
                all_akurasi = [data['akurasi_persen'] for data in self.test_data.values()]
                all_waktu_proses = [data['waktu_proses_ms'] for data in self.test_data.values()]
                all_tingkat_error = [data['tingkat_error'] for data in self.test_data.values()]
                all_success_rates = [data['success_rate_persen'] for data in self.test_data.values()]
                
                f.write(f"\nRINGKASAN STATISTIK TERINTEGRASI:\n")
                f.write("-" * 50 + "\n")
                f.write("🔬 JURNAL METRICS AVERAGES:\n")
                f.write(f"  Rata-rata Detection Rate:  {np.mean(all_detection_rates):.2f}%\n")
                f.write(f"  Rata-rata Accuracy:        {np.mean(all_accuracies):.2f}%\n")
                f.write(f"  Rata-rata Position Error:  {np.mean(all_position_errors):.4f} cm\n")
                f.write(f"  Rata-rata Angle Error:     {np.mean(all_angle_errors):.6f}°\n")
                f.write(f"  Rata-rata Processing Time: {np.mean(all_processing_times):.6f} s\n")
                f.write("📊 EXISTING SYSTEM AVERAGES:\n")
                f.write(f"  Rata-rata Akurasi:         {np.mean(all_akurasi):.2f}%\n")
                f.write(f"  Rata-rata Waktu Proses:    {np.mean(all_waktu_proses):.2f} ms\n")
                f.write(f"  Rata-rata Tingkat Error:   {np.mean(all_tingkat_error):.3f}\n")
                f.write(f"  Rata-rata Success Rate:    {np.mean(all_success_rates):.2f}%\n")
            
            f.write(f"\n" + "="*100 + "\n")
            f.write("PERBANDINGAN KEDUA SISTEM:\n")
            f.write("-" * 50 + "\n")
            f.write("• Jurnal Accuracy vs Existing Akurasi: Metode perhitungan berbeda\n")
            f.write("• Processing Time(s) vs Waktu Proses(ms): Unit berbeda, konsep sama\n")
            f.write("• Position/Angle Error: Data tambahan dari jurnal\n")
            f.write("• Success Rate: Mirip dengan Detection Rate tapi metode berbeda\n")
            f.write("• TP/FP/FN/TN: Detail confusion matrix untuk analisis mendalam\n\n")
            
            f.write("GENERATED BY: Integrated Hough Transform Monitoring Node\n")
            f.write("KOMPATIBEL DENGAN: Existing Data Collection System\n")
            f.write("UNTUK: BAB 4 Tugas Akhir - Analisis Performa Algoritma Lengkap\n")
            f.write("="*100 + "\n")
    
    def run(self):
        """Keep monitoring running"""
        rospy.spin()

if __name__ == '__main__':
    try:
        monitor = HoughMonitoringIntegrated()
        monitor.run()
    except rospy.ROSInterruptException:
        pass

#pragma once

//#include "../include/librealsense2/hpp/rs_frame.hpp"
//#include "../include/librealsense2/hpp/rs_types.hpp"
//#include "../include/librealsense2/hpp/rs_options.hpp"
#include "../include/librealsense2/hpp/rs_processing.hpp"

// ==============================
// rs_processing.h 
// ==============================

// ==============================
// rs_processing.hpp 
// ==============================

// ==============================
// hole-filter-with-color.h
// ==============================
#include "proc/synthetic-stream.h"

namespace librealsense
{

class hole_filter_with_color : public generic_processing_block
{
public:
	hole_filter_with_color() : generic_processing_block( "HoleFilterWithColor" )
	{
	}

protected:
	bool should_process( const rs2::frame& frame ) override;
	rs2::frame process_frame( const rs2::frame_source& soruce, const rs2::frame& f ) override;

	rs2::frame prepare_target_frame( const rs2::frame& f, const rs2::frame_source& source, rs2_extension tgt_type );

private:

	// カーネルサイズごとの距離の分散(標準偏差σの二乗)
	float sigma_space_pow2[5] ={ 0.444f, 1.12f, 2.811f, 6.829f, 15.122 };

	/** sRGBからCIE 1976 L*a*b*への変換
	* \param[in] rgb 変換元 画像すべてのRGBを持つ配列
	* \param[in,out] lab 変換先 画像すべてのL*a*b*値を持つ配列
	* \param[in] 画像のサイズ(width * height)
	*/
	void convert_rgb8_to_lab( const uint8_t* rgb, float* lab, const uint32_t size );

	/** sRGBからCIE XYZへの変換
	* \param[in] rgb 一画素のRGBの配列
	* \param[in,out] x Xの値
	* \param[in,out] y Yの値
	* \param[in,out] z Zの値
	*/
	void convert_rgb8_to_xyz( const uint8_t* rgb, float& x, float& y, float& z );

	/** ガンマ補正された値を戻す
	* \param[in] u sRGBのどれか1色の値
	* \return ガンマ展開した値
	*/
	float gamma_expanded( const float u );

	/** L*a*b*値同士の距離を求める
	* \param[in] r_lab 長さ3の配列
	* \param[in] l_lab 長さ3の配列
	* \return 距離
	*/
	float lab_distance( const float* r_lab, const float* l_lab );

	/** 指定座標におけるカーネル処理
	* \param[out] new_depth 出力値
	* \param[in] depth_image 深度画像データ
	* \param[in] lab_image 色画像データ
	* \param[in] x 対称x座標
	* \param[in] y 対称y座標
	*/
	void kernel_process( uint16_t& new_depth, const uint16_t* depth_image, const float* lab_image, const int x, const int y );


};

}
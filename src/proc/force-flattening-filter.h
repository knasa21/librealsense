#pragma once

#include "../include/librealsense2/hpp/rs_processing.hpp"

#include "proc/synthetic-stream.h"

#include <iostream>
#include <chrono>
#include <memory>
#include <forward_list>
#include <deque>
#include <list>

namespace librealsense
{

class force_flattening_filter : public generic_processing_block
{
public:
	force_flattening_filter() : generic_processing_block( "ForceFlatteningFilter" )
	{
	}

protected:
	bool should_process( const rs2::frame& frame ) override;
	virtual rs2::frame process_frame( const rs2::frame_source& soruce, const rs2::frame& f ) override;

	virtual rs2::frame prepare_target_frame( const rs2::frame& f, const rs2::frame_source& source, rs2_extension tgt_type );

	virtual void reset_cache() {};

	// カーネルサイズごとの距離の分散(標準偏差σの二乗)
	float _sqr_space_sigma_array[5] ={ 0.444f, 1.12f, 2.811f, 6.829f, 15.122 };

	std::chrono::system_clock::time_point _chrono_start, _chrono_end;

	std::shared_ptr<float> _lab_data;

	// 変換後深度、カラー
	std::shared_ptr<rs2::frame> _tgt_depth;
	std::shared_ptr<rs2::frame> _tgt_color;

	// ラベル用配列
	std::unique_ptr<uint32_t[]> _labels;

	// 色距離保存配列
	std::unique_ptr<float[]> _lab_distances;

	// 画像サイズ
	uint16_t _width, _height;

	// クラスタリスト
	std::vector<std::vector<uint32_t>> _clusters;

	/** ラベリングを行う
	* \param[in]  lab_image 探索元となるカラー画像
	* \param[in,out] labeled_indices ラベルがつけられたインデックスのリスト
	* \param[in] label_id 今回付けるラベルのID
	* \param[in] start_index 最初に選ぶピクセルのインデックス
	* \return クラスタのサイズ
	*/
	int labeling_process(
		const float* lab_image,
		std::vector<uint32_t>& labeled_indices,
		const uint32_t label_id,
		const uint32_t start_index,
		const float threshold
		);

	/** ラベルのつけられたポイントの平面化 
	* \param[in] depth_image 深度データ
	* \param[in,out] flat_depth_image 平面化した深度データ
	* \param[in,out] normal_image 平面部分の法線データ
	* \param[in] clusters クラスタ毎のインデックス配列
	*/
	void flattening(
		const uint16_t* depth_image,
		uint16_t* flat_depth_image,
		uint8_t* normal_image,
		std::vector<std::vector<uint32_t>>& clusters
		);

	/** RANSACによる平面推定
	* \param[in] depth_image 深度データ
	* \param[in] indices 使用するインデックス配列
	* \param[in, out] coefficients 平面の式の係数
	*/
	void ransac_plane(
		const uint16_t* depth_image,
		const std::vector<uint32_t>& indices,
		std::vector<float>& coefficients
	);

	/** sRGBからCIE 1976 L*a*b*への変換
	* \param[in] rgb 変換元 画像すべてのRGBを持つ配列
	* \param[in,out] lab 変換先 画像すべてのL*a*b*値を持つ配列
	* \param[in] width 画像横幅 
	* \param[in] height 画像縦幅 
	*/
	virtual void convert_rgb8_to_lab( const uint8_t* rgb, float* lab, const uint16_t width, const uint16_t height );

	/** sRGBからCIE XYZへの変換
	* \param[in] rgb 一画素のRGBの配列
	* \param[in,out] x Xの値
	* \param[in,out] y Yの値
	* \param[in,out] z Zの値
	*/
	virtual void convert_rgb8_to_xyz( const uint8_t* rgb, float& x, float& y, float& z );

	/** ガンマ補正された値を戻す
	* \param[in] u sRGBのどれか1色の値
	* \return ガンマ展開した値
	*/
	virtual float gamma_expanded( const float u );

	/** L*a*b*値同士の距離を求める
	* \param[in] r_lab 長さ3の配列
	* \param[in] l_lab 長さ3の配列
	* \return 距離
	*/
	virtual float lab_distance( const float* r_lab, const float* l_lab );

	/** 穴埋めフィルタ本体
	* \param[out] new_depth 出力
	* \param[in] depth_image 深度画像データ
	* \param[in] lab_image 色画像データ
	* \param[in] kernel_w　カーネルの一片の半分の切り捨て
	* \param[in] width 横幅
	* \param[in] height 縦幅
	*/
	virtual void hole_filter_process(
		uint16_t* new_depth_image,
		const uint16_t* depth_image,
		const float* lab_image,
		const int kernel_w,
		const int width,
		const int height
	);

	/** 指定座標におけるカーネル処理
	* \param[out] new_depth 出力
	* \param[in] depth_image 深度画像データ
	* \param[in] lab_image 色画像データ
	* \param[in] kernel_w　カーネルの一片の半分の切り捨て
	* \param[in] x 対象x座標
	* \param[in] y 対象y座標
	*/
	virtual void kernel_process( uint16_t& new_depth, const uint16_t* depth_image, const float* lab_image, const int kernel_w, const int x, const int y );

	/** 分散を求める(ただし0を無視する)
	* \param[in] vals データ配列
	* \return 分散値
	*/
	virtual float calc_dispersion( const std::vector<float>& vals );

	/** 色距離の配列を作成
	* \param[in] lab_image lab画像
	* \param[in,out] distances 色距離配列 
	*/
	virtual void make_lab_distances( const float* lab_image, float* distances, const int width, const int height );

	/** 時間計測開始
	*/
	void timer_start();

	/** 時間計測終了、出力
	* \param[in] text 出力時追加テキスト
	*/
	void timer_end( const std::string& text );

	/** 範囲指定乱数生成器
	* \return min <= return <= max
	*/
	uint64_t rand_range( const int& min, const int& max );
};

}
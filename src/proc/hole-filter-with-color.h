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

	/** sRGB����CIE 1976 L*a*b*�ւ̕ϊ�
	* \param[in] rgb �ϊ��� �摜���ׂĂ�RGB�����z��
	* \param[in,out] lab �ϊ��� �摜���ׂĂ�L*a*b*�l�����z��
	* \param[in] �摜�̃T�C�Y(width * height)
	*/
	void convert_rgb8_to_lab( const uint8_t* rgb, float* lab, const uint32_t size );

	/** sRGB����CIE XYZ�ւ̕ϊ�
	* \param[in] rgb ���f��RGB�̔z��
	* \param[in,out] x X�̒l
	* \param[in,out] y Y�̒l
	* \param[in,out] z Z�̒l
	*/
	void convert_rgb8_to_xyz( const uint8_t* rgb, float& x, float& y, float& z );

	/** �K���}�␳���ꂽ�l��߂�
	* \param[in] u sRGB�̂ǂꂩ1�F�̒l
	* \return �K���}�W�J�����l
	*/
	float gamma_expanded( const float u );
};

}
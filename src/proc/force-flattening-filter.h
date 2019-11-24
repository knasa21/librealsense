#pragma once

#include "../include/librealsense2/hpp/rs_processing.hpp"

#include "proc/synthetic-stream.h"

#include <iostream>
#include <chrono>
#include <memory>

using namespace std;

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

	// �J�[�l���T�C�Y���Ƃ̋����̕��U(�W���΍��Ђ̓��)
	float _sqr_space_sigma_array[5] ={ 0.444f, 1.12f, 2.811f, 6.829f, 15.122 };

	chrono::system_clock::time_point _chrono_start, _chrono_end;

	std::shared_ptr<float> _lab_data;

	std::shared_ptr<rs2::frame> _tgt_depth;

	/** sRGB����CIE 1976 L*a*b*�ւ̕ϊ�
	* \param[in] rgb �ϊ��� �摜���ׂĂ�RGB�����z��
	* \param[in,out] lab �ϊ��� �摜���ׂĂ�L*a*b*�l�����z��
	* \param[in] width �摜���� 
	* \param[in] height �摜�c�� 
	*/
	virtual void convert_rgb8_to_lab( const uint8_t* rgb, float* lab, const uint16_t width, const uint16_t height );

	/** sRGB����CIE XYZ�ւ̕ϊ�
	* \param[in] rgb ���f��RGB�̔z��
	* \param[in,out] x X�̒l
	* \param[in,out] y Y�̒l
	* \param[in,out] z Z�̒l
	*/
	virtual void convert_rgb8_to_xyz( const uint8_t* rgb, float& x, float& y, float& z );

	/** �K���}�␳���ꂽ�l��߂�
	* \param[in] u sRGB�̂ǂꂩ1�F�̒l
	* \return �K���}�W�J�����l
	*/
	virtual float gamma_expanded( const float u );

	/** L*a*b*�l���m�̋��������߂�
	* \param[in] r_lab ����3�̔z��
	* \param[in] l_lab ����3�̔z��
	* \return ����
	*/
	virtual float lab_distance( const float* r_lab, const float* l_lab );

	/** �����߃t�B���^�{��
	* \param[out] new_depth �o��
	* \param[in] depth_image �[�x�摜�f�[�^
	* \param[in] lab_image �F�摜�f�[�^
	* \param[in] kernel_w�@�J�[�l���̈�Ђ̔����̐؂�̂�
	* \param[in] width ����
	* \param[in] height �c��
	*/
	virtual void hole_filter_process(
		uint16_t* new_depth_image,
		const uint16_t* depth_image,
		const float* lab_image,
		const int kernel_w,
		const int width,
		const int height
	);

	/** �w����W�ɂ�����J�[�l������
	* \param[out] new_depth �o��
	* \param[in] depth_image �[�x�摜�f�[�^
	* \param[in] lab_image �F�摜�f�[�^
	* \param[in] kernel_w�@�J�[�l���̈�Ђ̔����̐؂�̂�
	* \param[in] x �Ώ�x���W
	* \param[in] y �Ώ�y���W
	*/
	virtual void kernel_process( uint16_t& new_depth, const uint16_t* depth_image, const float* lab_image, const int kernel_w, const int x, const int y );

	/** ���U�����߂�(������0�𖳎�����)
	* \param[in] vals �f�[�^�z��
	* \return ���U�l
	*/
	virtual float calc_dispersion( const std::vector<float>& vals );

	/** ���Ԍv���J�n
	*/
	void timer_start();

	/** ���Ԍv���I���A�o��
	* \param[in] text �o�͎��ǉ��e�L�X�g
	*/
	void timer_end( const string& text );

};

}
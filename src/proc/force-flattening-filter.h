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

	// �J�[�l���T�C�Y���Ƃ̋����̕��U(�W���΍��Ђ̓��)
	float _sqr_space_sigma_array[5] ={ 0.444f, 1.12f, 2.811f, 6.829f, 15.122 };

	std::chrono::system_clock::time_point _chrono_start, _chrono_end;

	std::shared_ptr<float> _lab_data;

	// �ϊ���[�x�A�J���[
	std::shared_ptr<rs2::frame> _tgt_depth;
	std::shared_ptr<rs2::frame> _tgt_color;

	// ���x���p�z��
	std::unique_ptr<uint32_t[]> _labels;

	// �F�����ۑ��z��
	std::unique_ptr<float[]> _lab_distances;

	// �摜�T�C�Y
	uint16_t _width, _height;

	// �N���X�^���X�g
	std::vector<std::vector<uint32_t>> _clusters;

	/** ���x�����O���s��
	* \param[in]  lab_image �T�����ƂȂ�J���[�摜
	* \param[in,out] labeled_indices ���x��������ꂽ�C���f�b�N�X�̃��X�g
	* \param[in] label_id ����t���郉�x����ID
	* \param[in] start_index �ŏ��ɑI�ԃs�N�Z���̃C���f�b�N�X
	* \return �N���X�^�̃T�C�Y
	*/
	int labeling_process(
		const float* lab_image,
		std::vector<uint32_t>& labeled_indices,
		const uint32_t label_id,
		const uint32_t start_index,
		const float threshold
		);

	/** ���x���̂���ꂽ�|�C���g�̕��ʉ� 
	* \param[in] depth_image �[�x�f�[�^
	* \param[in,out] flat_depth_image ���ʉ������[�x�f�[�^
	* \param[in,out] normal_image ���ʕ����̖@���f�[�^
	* \param[in] clusters �N���X�^���̃C���f�b�N�X�z��
	*/
	void flattening(
		const uint16_t* depth_image,
		uint16_t* flat_depth_image,
		uint8_t* normal_image,
		std::vector<std::vector<uint32_t>>& clusters
		);

	/** RANSAC�ɂ�镽�ʐ���
	* \param[in] depth_image �[�x�f�[�^
	* \param[in] indices �g�p����C���f�b�N�X�z��
	* \param[in, out] coefficients ���ʂ̎��̌W��
	*/
	void ransac_plane(
		const uint16_t* depth_image,
		const std::vector<uint32_t>& indices,
		std::vector<float>& coefficients
	);

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

	/** �F�����̔z����쐬
	* \param[in] lab_image lab�摜
	* \param[in,out] distances �F�����z�� 
	*/
	virtual void make_lab_distances( const float* lab_image, float* distances, const int width, const int height );

	/** ���Ԍv���J�n
	*/
	void timer_start();

	/** ���Ԍv���I���A�o��
	* \param[in] text �o�͎��ǉ��e�L�X�g
	*/
	void timer_end( const std::string& text );

	/** �͈͎w�藐��������
	* \return min <= return <= max
	*/
	uint64_t rand_range( const int& min, const int& max );
};

}
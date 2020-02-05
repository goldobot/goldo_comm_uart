#include "goldo_comm/cobs_encoder.hpp"
#include <cstring>

using namespace goldo_comm;


	inline void CobsEncoder::inc_ptr(iterator& it)
	{
		it++;
		if (it == m_buffer.end())
		{
			it = m_buffer.begin();
		}
	}

	inline ptrdiff_t CobsEncoder::ptr_diff(iterator a, iterator b)
	{
		if (a >= b)
		{
			return a - b;
		}
		else
		{
			return m_buffer.size() - (b-a);
		}
	}


CobsEncoder::CobsEncoder() :
	m_read_ptr(m_buffer.begin()),
	m_write_ptr(m_buffer.begin() + 1),
	m_code_ptr(m_buffer.begin())
{

};

size_t CobsEncoder::max_encoded_size(size_t in_size)
{
	return in_size + in_size/254 + 1;
}

void CobsEncoder::encode(const uint8_t* in_ptr, size_t& in_size, uint8_t* out_ptr, size_t& out_size)
{
	const uint8_t* in_beg = in_ptr;
	const uint8_t* in_end = in_ptr + in_size;

	uint8_t* out_beg = out_ptr;
	uint8_t* out_end = out_ptr + out_size;

	while (in_ptr != in_end && out_ptr != out_end)
	{		
		uint8_t len_code = ptr_diff(m_write_ptr, m_code_ptr);

		if (len_code == 255)
		{
			*m_code_ptr = len_code;
			m_code_ptr = m_write_ptr;
			inc_ptr(m_write_ptr);
			continue;
		}

		uint8_t c = *in_ptr++;
		*m_write_ptr = c;

		if (c == 0)
		{
			*m_code_ptr = len_code;
			m_code_ptr = m_write_ptr;
		}
		inc_ptr(m_write_ptr);
	}

	in_size = in_ptr - in_beg;
	out_size = out_ptr - out_beg;
}
size_t CobsEncoder::flush(uint8_t* buffer, size_t buffer_size)
{
	if (m_code_ptr >= m_read_ptr)
	{
		size_t out_size = std::min<size_t>(m_code_ptr - m_read_ptr, buffer_size);
		memcpy(buffer, &*m_read_ptr, out_size);
		m_read_ptr += out_size;
		return out_size;
	}
	return 0;
}

void CobsEncoder::end_message()
{
	uint8_t len_code = ptr_diff(m_write_ptr, m_code_ptr);
	*m_code_ptr = len_code;
	*m_write_ptr = 0;
	inc_ptr(m_write_ptr);
	m_code_ptr = m_write_ptr;
	inc_ptr(m_write_ptr);
}

void CobsEncoder::encode_message(const uint8_t* in_ptr, size_t& in_size, uint8_t* out_ptr, size_t& out_size)
{
	size_t encoded_size = out_size;
	uint8_t* beg_ptr = out_ptr;
	encode(in_ptr, in_size, out_ptr, encoded_size);
	out_ptr += encoded_size;
	end_message();
	encoded_size = out_size - encoded_size;
	out_ptr += flush(out_ptr, encoded_size);
	out_size = out_ptr - beg_ptr;
}
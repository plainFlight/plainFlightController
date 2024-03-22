
/*
* DESCRIPTION: A faster implementation of invSqrt.
*/
float invSqrt(float x) 
{
  unsigned int i = 0x5F1F1412 - (*(unsigned int*)&x >> 1);
  float tmp = *(float*)&i;
  float y = tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
  return y;
}


/*
* DESCRIPTION: A faster implementation of atan2 when compared to C library code. Formatting left in original coding style of Author.
* volkansalma/atan2_approximation.c
* https://gist.github.com/volkansalma/2972237
* http://pubs.opengroup.org/onlinepubs/009695399/functions/atan2.html
* Volkan SALMA
*/
float fastAtan2(float y, float x)
{
  const float ONEQTR_PI = M_PI / 4.0;
	const float THRQTR_PI = 3.0 * M_PI / 4.0;
	float r, angle;
	float abs_y = fabs(y) + 1e-10f;      // kludge to prevent 0/0 condition

	if ( x < 0.0f )
	{
		r = (x + abs_y) / (abs_y - x);
		angle = THRQTR_PI;
	}
	else
	{
		r = (x - abs_y) / (x + abs_y);
		angle = ONEQTR_PI;
	}

	angle += (0.1963f * r * r - 0.9817f) * r;

	if ( y < 0.0f )
	{
      return( -angle );     // negate if in quad III or IV
  }
	else
  {
		return( angle );
  }
}


/*
* DESCRIPTION: A faster 32bit implementation of map().
* Changing from Arduino's 'long' 64bit map() to 32bit saves ~50us per main loop iteration on PlainFlight v1.1.0.
* Arduino original implementation but using int32_t with parentheses added for good measure.
*/
int32_t map32(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max) 
{
  return (((x - in_min) * (out_max - out_min)) / (in_max - in_min)) + out_min;
}

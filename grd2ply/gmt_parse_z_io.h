struct Z {	/* -Z[<flags>] */
	BOOLEAN active;
	BOOLEAN swab;
	BOOLEAN repeat[2];
	char format[2];
	char type;
	int skip;
};

int GMT_parse_z_io (char *txt, struct Z *z)
{
	int i, k = 0;

	/* BOOLEAN input:  currently unused */

	for (i = 0; txt[i]; i++) {	/* Loop over flags */

		switch (txt[i]) {

			/* These 4 cases will set the format orientation for input */

			case 'T':
			case 'B':
			case 'L':
			case 'R':
				if (k > 2) {
					fprintf (stderr, "%s: GMT SYNTAX ERROR -Z: Choose format from [TBLR][TBLR]!\n", GMT_program);
					return 1;
				}
				z->format[k++] = txt[i];
				break;

			/* Set this if file is periodic, is grid registered, but repeating column or row is missing from input */

			case 'x':
				z->repeat[GMT_X] = 1;
				break;

			case 'y':
				z->repeat[GMT_Y] = 1;
				break;

			/* Optionally skip the given number of bytes before reading data */

			case 's':
				i++;
				if (txt[i]) {
					z->skip = atoi (&txt[i]);
					while (txt[i] && isdigit ((int)txt[i])) i++;
					i--;
				}
				break;

			case 'w':
				z->swab = TRUE;
				break;

			/* Set read pointer depending on data format */

			case 'a':	/* ASCII */
			case 'c':	/* Binary signed char */
			case 'u':	/* Binary unsigned char */
			case 'h':	/* Binary short 2-byte integer */
			case 'H':	/* Binary unsigned short 2-byte integer */
			case 'i':	/* Binary 4-byte integer */
			case 'I':	/* Binary 4-byte unsigned integer */
			case 'l':	/* Binary 4(or8)-byte integer, machine dependent! */
			case 'f':	/* Binary 4-byte float */
			case 'd':	/* Binary 8-byte double */
				z->type = txt[i];
				break;

			default:
				fprintf (stderr, "%s: GMT SYNTAX ERROR -Z: %c not a valid modifier!\n", GMT_program, txt[i]);
				GMT_exit (EXIT_FAILURE);
				break;
		}
	}

	return (0);
}

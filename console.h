void tty_init(void);
void net_init(unsigned short port);
unsigned int tty_check_writable(int fd);

#ifdef WIN32
int tty_check_readable(int fd);
#else
int select_wrapper(int maxfd, fd_set* i, fd_set* o);
#endif

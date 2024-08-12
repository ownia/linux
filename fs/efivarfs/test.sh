umount /sys/firmware/efi/efivars
#rmmod efivarfs
#insmod efivarfs.ko
mount -t efivarfs,refresh efivarfs /sys/firmware/efi/efivars

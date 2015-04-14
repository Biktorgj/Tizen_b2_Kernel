%define BOARD_TIZEN_B2 tizen_b2
%define BOARD_TIZEN_B2_SMK_DIS tizen_b2_smk_dis

%define BOARD_OPT_TIMA_EN _tima_en

Name: linux-3.4-exynos3250
Summary: The Linux Kernel
Version: 1.0.0
Release: 1
License: GPL
Group: System Environment/Kernel
Vendor: The Linux Community
URL: http://www.kernel.org
Source0: %{name}-%{version}.tar.gz

BuildRoot: %{_tmppath}/%{name}-%{PACKAGE_VERSION}-root
Provides: linux-3.4
%define __spec_install_post /usr/lib/rpm/brp-compress || :
%define debug_package %{nil}

BuildRequires:  kernel-headers
BuildRequires:  lzop
BuildRequires:  binutils-devel
BuildRequires:  module-init-tools elfutils-devel
BuildRequires:	python
BuildRequires:	gcc
BuildRequires:	bash
BuildRequires:	system-tools
BuildRequires:	sec-product-features
ExclusiveArch:  %arm

%description
The Linux Kernel, the operating system core itself

%define BOARDS %{BOARD_TIZEN_B2}

%if 0%{?sec_product_feature_tima}
%define OPTS %{BOARD_OPT_TIMA_EN}
%else
%define OPTS ""
%endif

%{lua:
for targets in string.gmatch(rpm.expand("%{BOARDS}"), "[%w_-]+")
do
print("%package -n linux-3.4-exynos3250_"..targets.." \n")
print("License:        TO_BE_FILLED \n")
print("Summary:        Linux support headers for userspace development \n")
print("Group:          TO_BE_FILLED/TO_BE_FILLED \n")
print("Requires(post): coreutils \n")
print("\n")
print("%files -n linux-3.4-exynos3250_"..targets.." \n")
print("/var/tmp/kernel/mod_"..targets.." \n")
print("/var/tmp/kernel/kernel-"..targets.."/zImage \n")
print("/var/tmp/kernel/kernel-"..targets.."/zImage-recovery \n")
print("/usr/share/license/linux-3.4-exynos3250_"..targets.." \n")
print("\n")
print("%post -n linux-3.4-exynos3250_"..targets.." \n")
print("mv /var/tmp/kernel/mod_"..targets.."/lib/modules/* /lib/modules/. \n")
print("mv /var/tmp/kernel/kernel-"..targets.."/zImage /var/tmp/kernel/. \n")
print("mv /var/tmp/kernel/kernel-"..targets.."/zImage-recovery /var/tmp/kernel/. \n")
print("\n")
print("%description -n linux-3.4-exynos3250_"..targets.." \n")
print("This package provides the exynos3250_eur linux kernel image & module.img. \n")
print("\n")
print("%package -n linux-3.4-exynos3250_"..targets.."-debuginfo \n")
print("License:        TO_BE_FILLED \n")
print("Summary:        Linux support headers for userspace development \n")
print("Group:          TO_BE_FILLED/TO_BE_FILLED \n")
print("\n")
print("%files -n linux-3.4-exynos3250_"..targets.."-debuginfo \n")
print("/var/tmp/kernel/mod_"..targets.." \n")
print("/var/tmp/kernel/kernel-"..targets.." \n")
print("\n")
print("%description -n linux-3.4-exynos3250_"..targets.."-debuginfo \n")
print("This package provides the exynos3250_eur linux kernel's debugging files. \n")
end }

%package -n kernel-headers-3.4-exynos3250
License:        TO_BE_FILLED
Summary:        Linux support headers for userspace development
Group:          TO_BE_FILLED/TO_BE_FILLED
Provides:       kernel-headers
Obsoletes:      kernel-headers

%description -n kernel-headers-3.4-exynos3250
This package provides userspaces headers from the Linux kernel.  These
headers are used by the installed headers for GNU glibc and other system
 libraries.

%package -n kernel-devel-3.4-exynos3250
License:        GPL
Summary:        Linux support kernel map and etc for other package
Group:          System/Kernel

%description -n kernel-devel-3.4-exynos3250
This package provides kernel map and etc information.

%prep
%setup -q

%build
%if 0%{?tizen_build_binary_release_type_eng}
%define RELEASE_TYPE ENG
%else
%define RELEASE_TYPE USR
%endif

for i in %{BOARDS}; do
	target=$i"%{OPTS}"
	mkdir -p %_builddir/mod_$target
	make distclean

	./release_obs.sh $target %{RELEASE_TYPE}

	cp -f arch/arm/boot/zImage %_builddir/zImage.$target
	cp -f arch/arm/boot/zImage %_builddir/zImage-recovery.$target
	cp -f System.map %_builddir/System.map.$target
	cp -f .config %_builddir/config.$target
	cp -f vmlinux %_builddir/vmlinux.$target
	make modules
	make modules_install INSTALL_MOD_PATH=%_builddir/mod_$target

	#remove all changed source codes for next build
	cd %_builddir
	rm -rf %{name}-%{version}
	/bin/tar -zxf %{SOURCE0}
	cd %{name}-%{version}
done

%install
mkdir -p %{buildroot}/usr
make mrproper
make headers_check
make headers_install INSTALL_HDR_PATH=%{buildroot}/usr

find  %{buildroot}/usr/include -name ".install" | xargs rm -f
find  %{buildroot}/usr/include -name "..install.cmd" | xargs rm -f
rm -rf %{buildroot}/usr/include/scsi
rm -f %{buildroot}/usr/include/asm*/atomic.h
rm -f %{buildroot}/usr/include/asm*/io.h

mkdir -p %{buildroot}/usr/share/license

for i in %{BOARDS}; do
	target=$i"%{OPTS}"

	mkdir -p %{buildroot}/var/tmp/kernel/kernel-$i
	mv %_builddir/mod_$target %{buildroot}/var/tmp/kernel/mod_$i

	mv %_builddir/zImage.$target %{buildroot}/var/tmp/kernel/kernel-$i/zImage
	mv %_builddir/zImage-recovery.$target %{buildroot}/var/tmp/kernel/kernel-$i/zImage-recovery

	mv %_builddir/System.map.$target %{buildroot}/var/tmp/kernel/kernel-$i/System.map
	mv %_builddir/config.$target %{buildroot}/var/tmp/kernel/kernel-$i/config
	mv %_builddir/vmlinux.$target %{buildroot}/var/tmp/kernel/kernel-$i/vmlinux

	cp -vf COPYING %{buildroot}/usr/share/license/%{name}_$i
done

find %{buildroot}/var/tmp/kernel/ -name 'System.map' > develfiles.pre # for secure storage
find %{buildroot}/var/tmp/kernel/ -name 'vmlinux' >> develfiles.pre   # for TIMA
find %{buildroot}/var/tmp/kernel/ -name '*.ko' >> develfiles.pre      # for TIMA
cat develfiles.pre | sed -e "s#%{buildroot}##g" | uniq | sort > develfiles

%clean
rm -rf %_builddir

%files -n kernel-headers-3.4-exynos3250
/usr/include/*

%files -n kernel-devel-3.4-exynos3250 -f develfiles

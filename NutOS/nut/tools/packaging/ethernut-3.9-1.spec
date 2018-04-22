##
## $Id: ethernut-3.9-1.spec 537 2004-09-12 09:14:44Z haraldkipp $
##

Summary: RTOS and TCP/IP stack for embedded systems.
Name: ethernut
Version: 3.9
Release: 1
Copyright: BSD
Group: Development/Tools
URL: http://www.ethernut.de/
Source: http://www.ethernut.de/arc/ethernut-3.9-1.src.tar.gz
BuildRoot: /var/tmp/%{name}-buildroot

%description
Nut/OS is an intentionally simple RTOS for tiny embedded
systems. It includes Nut/Net, the TCP/IP stack.

%prep
%setup

%build

%install
rm -rf $RPM_BUILD_ROOT
mkdir $RPM_BUILD_ROOT
mkdir $RPM_BUILD_ROOT/opt
mkdir $RPM_BUILD_ROOT/opt/ethernut
cp -rf ./nut/ $RPM_BUILD_ROOT/opt/ethernut/

%clean
rm -rf $RPM_BUILD_ROOT/opt/ethernut/

%files
/opt/ethernut/nut/


%post 
cd /opt/ethernut/nut/tools/crurom/
make install
cd /opt/ethernut/nut/tools/nutconf/
make install

echo "###############################################################"
echo "#  Nut/OS has been installed.                                 #"
echo "#                                                             #"
echo "#  Change to /opt/ethernut/ and run 'nut/tools/linux/nutconf' #"
echo "###############################################################"

%postun 

%changelog

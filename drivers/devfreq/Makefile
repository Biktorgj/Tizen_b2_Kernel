obj-$(CONFIG_PM_DEVFREQ)	+= devfreq.o
obj-$(CONFIG_DEVFREQ_GOV_SIMPLE_ONDEMAND)	+= governor_simpleondemand.o
obj-$(CONFIG_DEVFREQ_GOV_EXYNOS_BUS)       += governor_exynos_bus.o
obj-$(CONFIG_DEVFREQ_GOV_SIMPLE_USAGE)	+= governor_simpleusage.o
obj-$(CONFIG_DEVFREQ_GOV_PERFORMANCE)	+= governor_performance.o
obj-$(CONFIG_DEVFREQ_GOV_PM_QOS)	+= governor_pm_qos.o
obj-$(CONFIG_DEVFREQ_GOV_POWERSAVE)	+= governor_powersave.o
obj-$(CONFIG_DEVFREQ_GOV_USERSPACE)	+= governor_userspace.o

# DEVFREQ Drivers
ifdef CONFIG_ARM_EXYNOS5420_BUS_DEVFREQ
obj-y					+= exynos_ppmu.o
else ifdef CONFIG_ARM_EXYNOS5250_BUS_DEVFREQ
obj-y					+= exynos_ppmu.o
else ifdef CONFIG_ARM_EXYNOS3250_BUS_DEVFREQ
obj-y					+= exynos_ppmu.o
endif
obj-$(CONFIG_ARM_EXYNOS4_BUS_DEVFREQ)	+= exynos4_bus.o
obj-$(CONFIG_ARM_EXYNOS5250_BUS_DEVFREQ)	+= exynos5250_bus_mif.o exynos5250_bus_int.o
obj-$(CONFIG_ARM_EXYNOS5250_BUS_DEVFREQ)	+= exynos5_ppmu.o
obj-$(CONFIG_ARM_EXYNOS5260_BUS_DEVFREQ)	+= exynos5260_bus_mif.o exynos5260_bus_int.o exynos5260_bus_disp.o
obj-$(CONFIG_ARM_EXYNOS5260_BUS_DEVFREQ)	+= exynos5260_ppmu.o exynos_ppmu2.o
obj-$(CONFIG_ARM_EXYNOS5410_BUS_DEVFREQ)	+= exynos5410_bus_int.o exynos5410_bus_mif.o exynos5410_volt_ctrl.o
obj-$(CONFIG_ARM_EXYNOS5410_BUS_DEVFREQ)	+= noc_probe.o
obj-$(CONFIG_ARM_EXYNOS5420_BUS_DEVFREQ)	+= exynos5420_bus_mif.o exynos5420_bus_int.o noc_probe.o
obj-$(CONFIG_ARM_EXYNOS5420_BUS_DEVFREQ)	+= exynos5420_ppmu.o
obj-$(CONFIG_ARM_EXYNOS3470_BUS_DEVFREQ)	+= exynos3470_bus_mif.o exynos3470_bus_int.o exynos3470_ppmu.o
obj-$(CONFIG_ARM_EXYNOS3250_BUS_DEVFREQ)	+= exynos3250_bus_mif.o exynos3250_bus_int.o exynos3250_ppmu.o
obj-$(CONFIG_ARM_EXYNOS4415_BUS_DEVFREQ)	+= exynos4415_bus_mif.o exynos4415_bus_int.o exynos4415_ppmu.o
obj-$(CONFIG_SAMSUNG_NOCP_MONITOR)	+= nocp_monitor.o

#include <linux/init.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/io.h>
#include <linux/clk-provider.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <mach/ast2400.h>

// XXX TEMP HACKERY
//
// To be replaced by proper clock, pinmux and syscon drivers operating
// from DT parameters

static void __init aspeed_dt_init(void)
{
	of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);
}

static const struct of_device_id aspeed_clk_match[] __initconst = {
	{
		.compatible = "fixed-clock",
		.data = of_fixed_clk_setup,
	},
	{}
};

void __init aspeed_clk_init(void __iomem *base)
{
	of_clk_init(aspeed_clk_match);
}

#define AST_IO_VA	0xf0000000
#define AST_IO_PA	0x1e600000
#define AST_IO_SZ	0x00200000

#define AST_IO(__pa)	((void __iomem *)(((__pa) & 0x001fffff) | AST_IO_VA))

static struct map_desc aspeed_io_desc[] __initdata __maybe_unused = {
	{
		.virtual	=  AST_IO_VA,
		.pfn		= __phys_to_pfn(AST_IO_PA),
		.length		= AST_IO_SZ,
		.type		= MT_DEVICE
	},
};


#define UART_RBR	0
#define UART_IER	1
#define UART_FCR	2
#define UART_LCR	3
#define UART_MCR	4
#define UART_LSR	5
#define UART_MSR	6
#define UART_SCR	7
#define UART_THR	UART_RBR
#define UART_IIR	UART_FCR
#define UART_DLL	UART_RBR
#define UART_DLM	UART_IER
#define UART_DLAB	UART_LCR

#define LSR_DR   0x01  /* Data ready */
#define LSR_OE   0x02  /* Overrun */
#define LSR_PE   0x04  /* Parity error */
#define LSR_FE   0x08  /* Framing error */
#define LSR_BI   0x10  /* Break */
#define LSR_THRE 0x20  /* Xmit holding register empty */
#define LSR_TEMT 0x40  /* Xmitter empty */
#define LSR_ERR  0x80  /* Error */

#define LCR_DLAB 0x80

static void udbg_uart_out(unsigned int reg, u8 data)
{
	writeb(data, AST_IO(0x1E78F000 + reg * 4));
}

static u8 udbg_uart_in(unsigned int reg)
{
	return readb(AST_IO(0x1E78F000 + reg * 4)) & 0xff;
}

static void udbg_uart_setup(unsigned int speed, unsigned int clock)
{
	unsigned int dll, base_bauds;

	if (clock == 0)
		clock = 1843200;
	if (speed == 0)
		speed = 9600;

	base_bauds = clock / 16;
	dll = base_bauds / speed;

	printk("DLL=%d\n", dll);
	udbg_uart_out(UART_LCR, 0x00);
	udbg_uart_out(UART_IER, 0xff);
	udbg_uart_out(UART_IER, 0x00);
	udbg_uart_out(UART_LCR, LCR_DLAB);
	udbg_uart_out(UART_DLL, dll & 0xff);
	udbg_uart_out(UART_DLM, dll >> 8);
	/* 8 data, 1 stop, no parity */
	udbg_uart_out(UART_LCR, 0x3);
	/* RTS/DTR */
	udbg_uart_out(UART_MCR, 0x3);
	/* Clear & enable FIFOs */
	udbg_uart_out(UART_FCR, 0x7);
}

static void udbg_uart_flush(void)
{
	/* wait for idle */
	while ((udbg_uart_in(UART_LSR) & LSR_THRE) == 0)
		;
}

static void udbg_uart_putc(char c)
{
	if (c == '\n')
		udbg_uart_putc('\r');
	udbg_uart_flush();
	udbg_uart_out(UART_THR, c);
}

#define SCU_PASSWORD	0x1688A8A8

static void __init aspeed_init_early(void)
{
	u32 reg;

	// XXX UART stuff to fix to pinmux & co
	printk("UART IO MUX...\n");
	writel(0x02010023, AST_IO(AST_BASE_LPC | 0x9c));
	printk("UART PIN MUX...\n");
	writel(SCU_PASSWORD, AST_IO(AST_BASE_SCU)); // UNLOCK SCU
	printk("SCU LOCK: %08x\n", readl(AST_IO(AST_BASE_SCU)));
	writel(0xcb000000, AST_IO(AST_BASE_SCU | 0x80));
	writel(0x00fff0c0, AST_IO(AST_BASE_SCU | 0x84));
	writel(0x10CC5E80, AST_IO(AST_BASE_SCU | 0x0c));

	/* We enable the UART clock divisor in the SCU's misc control
	 * register, as the baud rates in aspeed.dtb all assume that the
	 * divisor is active
	 */
	reg = readl(AST_IO(AST_BASE_SCU | 0x2c));
	writel(reg | 0x00001000, AST_IO(AST_BASE_SCU | 0x2c));

	printk("DONE, MUX=%08x %08x\n", readl(AST_IO(AST_BASE_SCU | 0x80)),
	       readl(AST_IO(AST_BASE_SCU | 0x84)));
	printk("CLOCK_CTRL=%08x\n", readl(AST_IO(AST_BASE_SCU)));
	printk("WDT0C=%08x\n", readl(AST_IO(AST_BASE_WDT | 0x0c)));
	writel(0, AST_IO(AST_BASE_WDT | 0x0c));
	printk("WDT2C=%08x\n", readl(AST_IO(AST_BASE_WDT | 0x2c)));
	writel(0, AST_IO(AST_BASE_WDT | 0x2c));
	udbg_uart_setup(115200,0);
	udbg_uart_putc('F');
	udbg_uart_putc('O');
	udbg_uart_putc('O');
	udbg_uart_putc('\n');
}

static void aspeed_restart(enum reboot_mode mode, const char *cmd)
{
	// XXX Move that to WDT driver
	writel(0x0010, AST_IO(AST_BASE_WDT | 0x04));
	writel(0x0003, AST_IO(AST_BASE_WDT | 0x0c));
	writel(0x4755, AST_IO(AST_BASE_WDT | 0x08));
	for (;;);
}

static void __init aspeed_map_io(void)
{
	iotable_init(aspeed_io_desc, ARRAY_SIZE(aspeed_io_desc));
	debug_ll_io_init();

	printk("SOC Rev: %08x\n", readl(AST_IO(AST_BASE_SCU | 0x7c)));
}

static const char *const aspeed_dt_match[] __initconst = {
	"aspeed,ast2400",
	NULL,
};

DT_MACHINE_START(aspeed_dt, "ASpeed SoC")
	.map_io		= aspeed_map_io,
	.init_early	= aspeed_init_early,
	.init_machine	= aspeed_dt_init,
	.dt_compat	= aspeed_dt_match,
	.restart	= aspeed_restart,
MACHINE_END

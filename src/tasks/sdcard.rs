use defmt::*;
use {defmt_rtt as _, panic_probe as _};
use embassy_stm32::mode::Async;
use embassy_stm32::spi::Spi;
use embassy_stm32::spi::mode::Master;
use embedded_sdmmc::{VolumeIdx, Mode};

type MySpi = Spi<'static, Async, Master>;
type MyCsPin = embassy_stm32::gpio::Output<'static>; // O tipo Output padrão já assume o pino correto internamente

// Agora montamos o Type Alias de forma limpa:
pub type MyVolumeManager = embedded_sdmmc::VolumeManager<
    embedded_sdmmc::SdCard<
        embedded_hal_bus::spi::ExclusiveDevice<
            MySpi,
            MyCsPin,
            embassy_time::Delay,
        >,
        embassy_time::Delay,
    >,
    crate::DummyTimeSource, // Ajuste o caminho aqui se necessário
>;

#[embassy_executor::task]
pub async fn sd_task(
    volume_mgr: MyVolumeManager) {

    let volume0;
    match volume_mgr.open_volume(VolumeIdx(0)) {
        Ok(vol) => {
            volume0 = vol;
            info!("Volume 0 opened successfully.");
        },
        Err(_e) => {
            return; // Sai da task se não conseguir abrir o volume
        }
    }
    //info!("Volume 0: {:?}", volume0);

    // Open the root directory (mutably borrows from the volume).
    let root_dir = match volume0.open_root_dir() {
    Ok(dir) => {
        info!("Diretório raiz aberto com sucesso.");
        dir // Retorna o dir para a variável root_dir
    },
    Err(_e) => {
        defmt::error!("Erro ao abrir diretório raiz.");
        return; // Sai da task se falhar
    }
};
    // Open a file called "MY_FILE.TXT" in the root directory
    // This mutably borrows the directory.
    let my_file = match root_dir.open_file_in_dir("TESTE.TXT", Mode::ReadOnly) {
    Ok(file) => {
        defmt::info!("Arquivo aberto com sucesso!");
        file // Retorna o arquivo para a variável my_file
    },
    Err(_e) => {
        defmt::error!("Erro ao abrir o arquivo.");
        return; // Sai da task se o arquivo não existir ou falhar
    }
};
    // Print the contents of the file, assuming it's in ISO-8859-1 encoding
    while !my_file.is_eof() {
        let mut buffer = [0u8; 512];
        
        // Tentamos ler o arquivo. Se der certo, 'num_read' recebe a quantidade de bytes lidos
        if let Ok(num_read) = my_file.read(&mut buffer) {
            // Se leu mais de 0 bytes, processa o buffer
            for b in &buffer[0..num_read] {
                info!("{}", *b as char);
            }
        } else {
            // Se caiu aqui, houve um erro físico ou lógica na leitura do cartão
            defmt::error!("Falha crítica ao ler os dados do arquivo no cartão SD.");
            break; // Sai do loop 'while' com segurança sem derrubar o microcontrolador
        }
    }
    loop {
        embassy_time::Timer::after_secs(5).await;

        defmt::info!("Task acessando o SD diretamente (sem Mutex)...");
    }
}
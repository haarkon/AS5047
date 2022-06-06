#include "mbed.h"
#include "AS5047.h"

AS5047::AS5047 (PinName mosi, PinName miso, PinName sck, PinName ss, int freq) {
    _sensor = new SPI (mosi, miso, sck, ss);
    _sensor->frequency(freq);
    _sensor->format(16,1);   
}

// Fonction de calcul de la parité paire du message
int AS5047::ComputeEvenParity (short value){
    int sum=0;

    // La parité se calcules sur les X bits de du message
    // Les trames de commande (envoyées ou reçues) : 15 bits de poids faible => utilisation de AS5047_15BITMASK
    // Les trames de données (envoyées)            : 15 bits de poids faible => utilisation de AS5047_15BITMASK
    // Les trames de données (reçues)              : 16 bits

    for (char i=0; i<16; i++) {
        sum ^= (value >> i) & 0x0001;
    }
    return sum;
}

// Fonction de vérification que l'adresse est valide
bool AS5047::isAddressValid (short address){
    // Simple comparaison avec toutes les adresses des registres (vu qu'il n'y a pas vraiment de formule pour tr    aiter tous les cas en 1 seule fois)
    if ((address == AS5047_ADR_NOP) || (address == AS5047_ADR_ERRFL) || (address == AS5047_ADR_PROG) || (address == AS5047_ADR_DIAAGC) || (address == AS5047_ADR_MAG) || (address == AS5047_ADR_ANGLEUNC) || (address == AS5047_ADR_ANGLECOM) || (address == AS5047_ADR_ZPOSM) || (address == AS5047_ADR_ZPOSL) || (address == AS5047_ADR_SETTING1) || (address == AS5047_ADR_SETTING2)) return true;
    else return false;
}

// Fonction de vérification que la donnée n'est pas sur plus de 14 bits
bool AS5047::isDataValid (short value){
    if (value & 0xC000) return false;
    else return true;
}

// Fonction de vérification d'un paquet (théoriquement à n'utiliser qu'en réception)
AS5047::T_AS5047_Error AS5047::isPaquetValid (T_packet packet){
    // On commence par effacer les flags d'erreur
    T_AS5047_Error cr = AS5047_ERR_OK;

    // On vérifie maintenant la parité du message.
    // Le masque doit avoir été appliqué. Tous les bits seront testés.
    if (ComputeEvenParity(packet.word) != 0)    cr |= AS5047_ERR_PARITY;            // Erreur de parité
    if (packet.Bits.flag != 0) {                                                    // Si le flag est actif
                                                cr |= AS5047_ERR_FLAG;              // Erreur de FLAG
                                                cr |= diagnose(&AS5047_diagnostic); // Execution du programme de diagnostic (automatique)
    }
    return cr;
}

// Fonction de creation d'un paquet à partir des paramètres fournis
AS5047::T_AS5047_Error AS5047::createFrame (T_packet *packet, short value, T_direction direction){
    
    // Vérification des paramètres transmis à la fonction
    if (packet == nullptr)      return AS5047_ERR_PARAM;
    if (!isDataValid(value))    return AS5047_ERR_DATA;

    // Création du paquet
    packet->Bits.data = value;
    packet->Bits.flag = (short)direction;
    packet->Bits.parity = ComputeEvenParity (packet->word & AS5047_15BITMASK);
    return AS5047_ERR_OK;
}

// Fonction de diagnostic de l'AS5047
AS5047::T_AS5047_Error AS5047::diagnose (T_AS5047_DIAG *value) {
    T_AS5047_Error  cr;
    T_DIAAGC_REG    diag;
    T_ERRFL_REG     errfl;

    if (value == nullptr)       return AS5047_ERR_PARAM;                // Vérification des paramètres transmis à la fonction

    *value = AS5047_DIAG_OK;                                            // Effacement des erreurs précédente 

    cr = readData (AS5047_ADR_ERRFL, &errfl.packet.word);               // Lecture du registre ERROR FLAG
    if (cr == AS5047_ERR_OK) {                                          // Si la lecture s'est bien déroulée
        // traduction des flags actifs en erreur
        if (errfl.Bits.PARERR)      *value |= AS5047_DIAG_PARITY;       
        if (errfl.Bits.INVCOMM)     *value |= AS5047_DIAG_INVCOMM;
        if (errfl.Bits.FRERR)       *value |= AS5047_DIAG_FRAME;

        cr = readData (AS5047_ADR_DIAAGC, &diag.packet.word);           // Lecture du registre de diagnostic
        if (cr == AS5047_ERR_OK) {                                      // Si la lecture s'est bien déroulée
            // Traduction des flags actifs en erreur
            if (diag.Bits.MAGL)     *value += AS5047_DIAG_MAGL;
            if (diag.Bits.MAGH)     *value += AS5047_DIAG_MAGH;
            if (diag.Bits.COF)      *value += AS5047_DIAG_CORDIC;
            if (diag.Bits.LF)       *value += AS5047_DIAG_NOT_READY;
        }
    }
    return cr;
}

// Fonction d'écriture d'une valeur dans un registre
AS5047::T_AS5047_Error AS5047::writeData (short address, short value){
    T_AS5047_Error cr = AS5047_ERR_OK;
    T_packet TxPacket[3], RxPacket[3];
    
    // Vérification des paramètres transmis à la fonction
    if (!isAddressValid (address))  cr |= AS5047_ERR_ADDRESS;
    if (!isDataValid(value))        cr |= AS5047_ERR_DATA;

    if (cr != AS5047_ERR_OK)        return cr;

    createFrame (&TxPacket[0], address);                    // Trame 1 : Création de la trame de commande (write)
    createFrame (&TxPacket[1], value);                      // Trame 2 : Création de la trame de donnée (write)
    createFrame (&TxPacket[2], AS5047_ADR_NOP, dataRead);   // TRame 3 : Création d'une trame "NOP" pour permettre la lecture de la donnée retournée par l'AS5047

    // Transmission
    _sensor->select();
    _sensor->write (TxPacket[0].bytes,3,RxPacket[0].bytes,3);
    _sensor->deselect();

    // On a maintenant reçu 3 trames :
    //  - Trame 1 = NOP - On ne la traitera pas !
    //  - Trame 2 = valeur précédente du registre d'adresse "address"
    //  - Trame 3 = nouvelle valeur du registre 

    cr |= isPaquetValid (RxPacket[1]);
    cr |= isPaquetValid (RxPacket[2]);

    if (RxPacket[2].Bits.data != value)     cr |= AS5047_ERR_FAIL;
    return cr;
}

// Fonction de lecture des données
AS5047::T_AS5047_Error AS5047::readData (short address, short *value){
    T_AS5047_Error cr;
    T_packet TxPacket[2], RxPacket[2];

    // Vérification des paramètres transmis à la fonction
    if (!isAddressValid (address))  cr |= AS5047_ERR_ADDRESS;
    if (value == nullptr)           cr |= AS5047_ERR_PARAM;

    if (cr != AS5047_ERR_OK)        return cr;

    createFrame(&TxPacket[0], address, dataRead);           // Trame 1 : Création de la trame de commande (lecture de données)
    createFrame(&TxPacket[1], AS5047_ADR_NOP, dataRead);    // Trame 2 : Création d'une trame "NOP" pour permettre la lecture de la donnée retournée par l'AS5047

    // Transmission
    _sensor->select();
    _sensor->write (TxPacket[0].bytes,2,RxPacket[0].bytes,2);
    _sensor->deselect();

    // Validation
    cr = isPaquetValid(RxPacket[1]);
    if (cr != AS5047_ERR_OK)        return cr;

    *value = RxPacket[1].Bits.data;
    return AS5047_ERR_OK;
}

AS5047::T_AS5047_Error AS5047::setResolution (short resolution){
    T_AS5047_Error cr;
    T_SETTING1_REG setting1;
    T_SETTING2_REG setting2;

    cr = readData (AS5047_ADR_SETTING1, &setting1.packet.word);
    if (cr != AS5047_ERR_OK) return cr;

    cr = readData (AS5047_ADR_SETTING2, &setting2.packet.word);
    if (cr != AS5047_ERR_OK) return cr;

    switch (resolution){
        case 4000 : 
            setting2.Bits.ABIRES = AS5047_RES_4000;
            setting1.Bits.ABIBIN = 0;
            break;
        case 2000 : 
            setting2.Bits.ABIRES = AS5047_RES_2000;
            setting1.Bits.ABIBIN = 0;
            break;
        case 1600 : 
            setting2.Bits.ABIRES = AS5047_RES_1600;
            setting1.Bits.ABIBIN = 0;
            break;
        case 1200 : 
            setting2.Bits.ABIRES = AS5047_RES_1200;
            setting1.Bits.ABIBIN = 0;
            break;
        case 800 : 
            setting2.Bits.ABIRES = AS5047_RES_800;
            setting1.Bits.ABIBIN = 0;
            break;
        case 400 : 
            setting2.Bits.ABIRES = AS5047_RES_400;
            setting1.Bits.ABIBIN = 0;
            break;
        case 200 : 
            setting2.Bits.ABIRES = AS5047_RES_200;
            setting1.Bits.ABIBIN = 0;
            break;
        case 100 : 
            setting2.Bits.ABIRES = AS5047_RES_100;
            setting1.Bits.ABIBIN = 0;
            break;
        case 4096 : 
            setting2.Bits.ABIRES = AS5047_RES_4096;
            setting1.Bits.ABIBIN = 1;
            break;
        case 2048 : 
            setting2.Bits.ABIRES = AS5047_RES_2048;
            setting1.Bits.ABIBIN = 1;
            break;
        case 1024 : 
            setting2.Bits.ABIRES = AS5047_RES_1024;
            setting1.Bits.ABIBIN = 1;
            break;
        default :
            return AS5047_ERR_PARAM;
            break;
    }
    cr = writeData (AS5047_ADR_SETTING1,setting1.packet.word);
    if (cr != AS5047_ERR_OK) return cr;

    cr = writeData (AS5047_ADR_SETTING2,setting2.packet.word);
    if (cr != AS5047_ERR_OK) return cr;

    return AS5047_ERR_OK;
}

AS5047::T_AS5047_Error AS5047::setUVWPairPoles (short pairPoles){
    T_AS5047_Error cr;
    T_SETTING2_REG setting2;

    if ((pairPoles < 1) || (pairPoles > 7)) return AS5047_ERR_PARAM;

    cr = readData (AS5047_ADR_SETTING2, &setting2.packet.word);
    if (cr != AS5047_ERR_OK) return cr;

    setting2.Bits.UVWPP = pairPoles-1;

    cr = writeData (AS5047_ADR_SETTING2,setting2.packet.word);
    if (cr != AS5047_ERR_OK) return cr;

    return AS5047_ERR_OK;
}

AS5047::T_AS5047_Error AS5047::ActivatePWM (int port) {
    T_AS5047_Error  cr;
    T_SETTING1_REG  setting1;
    
    if ((port != 0) && (port != 1)) return AS5047_ERR_PARAM;

    cr = readData (AS5047_ADR_SETTING1, &setting1.packet.word);
    if (cr != AS5047_ERR_OK) return cr;
    setting1.Bits.UVW_ABI = port;
    setting1.Bits.PWMON = 1;
    cr = writeData (AS5047_ADR_SETTING1, setting1.packet.word);
    if (cr != AS5047_ERR_OK) return cr;
    return AS5047_ERR_OK;
}

AS5047::T_AS5047_Error AS5047::getDirection (int *direction) {
    T_AS5047_Error  cr;
    T_SETTING1_REG  setting1;
    
    if (direction == nullptr) return AS5047_ERR_PARAM;

    cr = readData (AS5047_ADR_SETTING1, &setting1.packet.word);
    if (cr != AS5047_ERR_OK) return cr;
    *direction = setting1.Bits.DIR;
    return AS5047_ERR_OK;
}

AS5047::T_AS5047_Error AS5047::isAS5047Ready (bool *ready) {
    T_AS5047_Error  cr;
    T_DIAAGC_REG    diag;
    
    if (ready == nullptr) return AS5047_ERR_PARAM;

    cr = readData (AS5047_ADR_DIAAGC, &diag.packet.word);
    if (cr != AS5047_ERR_OK) return cr;
    *ready = (bool)diag.Bits.LF;
    return AS5047_ERR_OK;
}

AS5047::T_AS5047_Error AS5047::getAngularPosition (short *position) {
    T_AS5047_Error  cr;
    T_packet        packet;

    if (position == nullptr) return AS5047_ERR_PARAM;

    cr = readData (AS5047_ADR_ANGLECOM, &packet.word);
    if (cr != AS5047_ERR_OK) return cr;
    *position = packet.Bits.data;
    return AS5047_ERR_OK;
}

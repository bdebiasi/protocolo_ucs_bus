# protocolo_ucs_bus
Protocolo de Comunicação UCS Bus

Projeto desenvolvido utilizando software STVD.

Proposta:

Detalhes do Protocolo:
- Protocolo de comunicação proprietário
- Duplex
- Comunicação:
    Mestre – Escravo
    Escravo – Escravo
- Topologia de Rede:
    Barramento
- Interface:
    RS485
- Protocolo de Comunicação:
  STX | Tam Pacote | End Destino | End Origem | Comando | Dados | BCC

Onde:

STX – Comando de início de envio do pacote, da tabela ASCII é o número 0x02
Tam Pacote – Número de Bytes do pacote
Destino – Endereço do elemento da rede que irá receber o pacote
Origem – Endereço do elemento da rede que originou o envio do pacote
Comando – Comando implementado no protocolo, que o receptor deverá tratar, no caso possuímos 6 comandos:
  0x1: Leitura do status do botão 1, 0 quando não estiver acionado e 1 quando estiver acionado
  0x2: Leitura do status do botão 2, 0 quando não estiver acionado e 1 quando estiver acionado
  0x3: Escrita no Led 1, 0 para desligar o LED e 1 para ligar o LED
  0x4: Escrita no Led 2, 0 para desligar o LED e 1 para ligar o LED
  0x5: Pisca Led1, primeiro byte o número de piscadas e o segundo byte o tempo de cada piscada
  0x6: Pisca Led2, primeiro byte o número de piscadas e o segundo byte o tempo de cada piscada
  0x7: Escreve uma mensagem do display, onde o primeiro dado é a posição do display (0x80 para a primeira posição) e os demais dados a mensagem (em ASCII)

Dados: Dados enviados no protocolo, lembrando que o retorno de uma mensagem sempre deverá iniciar os dados com 0x06 ACK (acknowledge) para sinalizar uma comunicação correta e 0x15 NAK (negative acknowledge) para sinalizar uma comunicação errada.
BCC: Byte verificador, sendo composto por um XOR entre todos os bytes (sem considerar o próprio BCC)
